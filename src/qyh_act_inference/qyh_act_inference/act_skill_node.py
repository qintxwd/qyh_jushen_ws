"""
ACT 技能节点 (ACT Skill Node)

将 ACT 推理作为任务引擎的一个技能节点，可以在行为树中调用

用法示例（任务 JSON）:
{
    "type": "ACTExecute",
    "params": {
        "model_name": "pickup_cube",
        "max_duration": 30.0,
        "action_scale": 0.4
    }
}
"""

import os
import time
from typing import Dict, Any, Optional
from enum import Enum

# 导入任务引擎基类
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'qyh_task_engine', 'qyh_task_engine'))

try:
    from qyh_task_engine.base_node import SkillNode, SkillStatus, SkillResult
except ImportError:
    # 如果直接导入失败，定义占位
    class SkillStatus(Enum):
        IDLE = "idle"
        RUNNING = "running"
        SUCCESS = "success"
        FAILURE = "failure"
        HALTED = "halted"
    
    class SkillResult:
        def __init__(self, status, message="", data=None):
            self.status = status
            self.message = message
            self.data = data or {}
    
    class SkillNode:
        NODE_TYPE = "BaseSkill"
        PARAM_SCHEMA = {}
        
        def __init__(self, node_id, params=None, **kwargs):
            self.node_id = node_id
            self.params = params or {}
            self.ros_node = kwargs.get('ros_node')
            self.status = SkillStatus.IDLE


class ACTExecuteNode(SkillNode):
    """
    ACT 模型执行节点
    
    调用 ACT 推理节点执行学习到的动作策略
    
    参数:
        model_name: 模型名称（对应 models/ 目录下的模型）
        model_path: 模型完整路径（可选，优先于 model_name）
        max_duration: 最大执行时间 (秒)，默认 30
        action_scale: 动作缩放因子，默认从配置读取
        success_threshold: 成功判定阈值（预留，用于未来的成功检测）
        stop_on_contact: 是否在检测到接触时停止（预留）
    
    依赖:
        - qyh_act_inference 包的 act_inference_node 必须运行
        - 或者直接在节点内加载模型（内嵌模式）
    """
    
    NODE_TYPE = "ACTExecute"
    
    PARAM_SCHEMA = {
        'action_id': {'type': 'string', 'required': False},  # 动作 ID (优先，从 model_actions 加载)
        'model_name': {'type': 'string', 'required': False},
        'model_path': {'type': 'string', 'required': False},
        'max_duration': {'type': 'float', 'default': 30.0},
        'action_scale': {'type': 'float', 'default': 0.4},
        'smoothing_alpha': {'type': 'float', 'default': 0.3},
        'success_threshold': {'type': 'float', 'default': 0.8},
        'stop_on_contact': {'type': 'bool', 'default': False},
        'use_external_node': {'type': 'bool', 'default': True},  # 是否使用外部推理节点
    }
    
    # 模型目录
    MODELS_DIR = os.path.expanduser("~/qyh-robot-system/models")
    MODEL_ACTIONS_DIR = os.path.expanduser("~/qyh-robot-system/model_actions")
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        
        # 服务客户端
        self._start_client = None
        self._stop_client = None
        self._load_model_client = None
        self._status_sub = None
        
        # 内嵌推理（如果不使用外部节点）
        self._policy = None
        self._executor = None
        
        # 状态
        self._is_executing = False
        self._start_time: Optional[float] = None
        self._inference_status = "unknown"
    
    def log_info(self, msg):
        if self.ros_node:
            self.ros_node.get_logger().info(msg)
        else:
            print(f"[INFO] {msg}")
    
    def log_warn(self, msg):
        if self.ros_node:
            self.ros_node.get_logger().warn(msg)
        else:
            print(f"[WARN] {msg}")
    
    def log_error(self, msg):
        if self.ros_node:
            self.ros_node.get_logger().error(msg)
        else:
            print(f"[ERROR] {msg}")
    
    def setup(self) -> bool:
        """初始化 ACT 执行节点"""
        self.log_info("="*50)
        self.log_info(f"[ACTExecute] Setup - ID: {self.node_id}")
        self.log_info(f"  Params: {self.params}")
        
        use_external = self.params.get('use_external_node', True)
        
        if use_external:
            return self._setup_external_mode()
        else:
            return self._setup_embedded_mode()
    
    def _setup_external_mode(self) -> bool:
        """设置外部推理节点模式"""
        self.log_info("  Mode: External inference node")
        
        if not self.ros_node:
            self.log_warn("  No ROS node, running in MOCK mode")
            return True
        
        try:
            from std_srvs.srv import SetBool, Trigger
            from std_msgs.msg import String
            
            # 创建服务客户端
            self._start_client = self.ros_node.create_client(
                SetBool,
                '/act_inference_node/start'
            )
            
            self._load_model_client = self.ros_node.create_client(
                Trigger,
                '/act_inference_node/load_model'
            )
            
            self._stop_client = self.ros_node.create_client(
                SetBool,
                '/act_inference_node/start'
            )
            
            # 订阅状态
            self._status_sub = self.ros_node.create_subscription(
                String,
                '/act_inference_node/status',
                self._status_callback,
                10
            )
            
            self.log_info("  Created service clients for act_inference_node")
            self.log_info("="*50)
            return True
            
        except Exception as e:
            self.log_error(f"  Failed to setup: {e}")
            return False
    
    def _setup_embedded_mode(self) -> bool:
        """设置内嵌推理模式（直接在技能节点内加载模型）"""
        self.log_info("  Mode: Embedded inference")
        
        try:
            from qyh_act_inference import ACTPolicy, ActionExecutor, InferenceConfig
            from qyh_act_inference.inference_config import get_right_arm_pickup_config
            
            # 获取模型路径
            model_path = self._resolve_model_path()
            if not model_path:
                self.log_error("  Model path not specified")
                return False
            
            # 创建配置
            config = get_right_arm_pickup_config()
            config.model_path = model_path
            config.action_scale = self.params.get('action_scale', 0.4)
            config.smoothing_alpha = self.params.get('smoothing_alpha', 0.3)
            
            # 加载模型
            self._policy = ACTPolicy(config)
            self._executor = ActionExecutor(config)
            
            if not self._policy.load():
                self.log_error("  Failed to load model")
                return False
            
            self.log_info(f"  Model loaded: {model_path}")
            self.log_info("="*50)
            return True
            
        except Exception as e:
            self.log_error(f"  Failed to setup embedded mode: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _resolve_model_path(self) -> Optional[str]:
        """解析模型路径"""
        # 优先使用完整路径
        if 'model_path' in self.params and self.params['model_path']:
            path = self.params['model_path']
            if os.path.exists(path):
                return path
            # 尝试展开 ~
            path = os.path.expanduser(path)
            if os.path.exists(path):
                return path
        
        # 使用 action_id 从 model_actions 目录加载
        if 'action_id' in self.params and self.params['action_id']:
            action_id = self.params['action_id']
            model_dir = os.path.join(self.MODEL_ACTIONS_DIR, action_id, 'model')
            # 检查各种模型文件格式
            for filename in ['policy_best.ckpt', 'policy.ckpt', 'policy.pt']:
                model_path = os.path.join(model_dir, filename)
                if os.path.exists(model_path):
                    self.log_info(f"  Found model from action: {model_path}")
                    return model_path
        
        # 使用模型名称
        if 'model_name' in self.params and self.params['model_name']:
            model_name = self.params['model_name']
            # 查找模型目录
            model_path = os.path.join(self.MODELS_DIR, model_name, 'policy.pt')
            if os.path.exists(model_path):
                return model_path
            # 也检查 ckpt 格式
            model_path = os.path.join(self.MODELS_DIR, model_name, 'policy.ckpt')
            if os.path.exists(model_path):
                return model_path
        
        return None
    
    def _status_callback(self, msg):
        """推理节点状态回调"""
        self._inference_status = msg.data
    
    def execute(self) -> SkillResult:
        """执行 ACT 推理"""
        max_duration = self.params.get('max_duration', 30.0)
        
        # Mock 模式
        if not self.ros_node and not self._policy:
            if not self._is_executing:
                self._is_executing = True
                self._start_time = time.time()
                self.log_info(f"[ACTExecute] Starting (mock mode)")
                return SkillResult(
                    status=SkillStatus.RUNNING,
                    message="Executing ACT policy (mock)"
                )
            
            # 模拟执行 3 秒
            elapsed = time.time() - self._start_time
            if elapsed > 3.0:
                self.log_info(f"[ACTExecute] Completed (mock)")
                return SkillResult(
                    status=SkillStatus.SUCCESS,
                    message="ACT execution completed (mock)"
                )
            
            return SkillResult(
                status=SkillStatus.RUNNING,
                message=f"Executing... {elapsed:.1f}s"
            )
        
        # 真实执行
        use_external = self.params.get('use_external_node', True)
        
        if use_external:
            return self._execute_external()
        else:
            return self._execute_embedded()
    
    def _execute_external(self) -> SkillResult:
        """使用外部推理节点执行"""
        max_duration = self.params.get('max_duration', 30.0)
        
        if not self._is_executing:
            # 首次调用：加载模型并启动
            self._is_executing = True
            self._start_time = time.time()
            
            # 先加载模型（如果需要）
            # TODO: 实现动态模型加载
            
            # 启动推理
            from std_srvs.srv import SetBool
            
            if not self._start_client.wait_for_service(timeout_sec=2.0):
                self.log_error("ACT inference service not available")
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message="ACT inference service not available"
                )
            
            request = SetBool.Request()
            request.data = True
            
            future = self._start_client.call_async(request)
            # 等待结果
            while not future.done():
                time.sleep(0.01)
            
            result = future.result()
            if not result.success:
                self.log_error(f"Failed to start inference: {result.message}")
                return SkillResult(
                    status=SkillStatus.FAILURE,
                    message=result.message
                )
            
            self.log_info(f"[ACTExecute] Started external inference")
            return SkillResult(
                status=SkillStatus.RUNNING,
                message="ACT inference started"
            )
        
        # 检查执行时间
        elapsed = time.time() - self._start_time
        
        if elapsed > max_duration:
            # 超时，停止执行
            self._stop_inference()
            self.log_warn(f"[ACTExecute] Timeout after {elapsed:.1f}s")
            return SkillResult(
                status=SkillStatus.SUCCESS,  # 超时也算完成
                message=f"ACT execution timeout after {elapsed:.1f}s"
            )
        
        # TODO: 检查成功条件（基于任务完成检测）
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Executing ACT... {elapsed:.1f}s / {max_duration}s"
        )
    
    def _execute_embedded(self) -> SkillResult:
        """使用内嵌模式执行"""
        max_duration = self.params.get('max_duration', 30.0)
        
        if not self._is_executing:
            self._is_executing = True
            self._start_time = time.time()
            self._policy.reset()
            self._executor.reset()
            self.log_info(f"[ACTExecute] Started embedded inference")
        
        elapsed = time.time() - self._start_time
        
        if elapsed > max_duration:
            self.log_info(f"[ACTExecute] Completed after {elapsed:.1f}s")
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=f"ACT execution completed after {elapsed:.1f}s"
            )
        
        # 执行一步推理（实际的推理循环在外部定时器中）
        # 这里只检查状态
        
        return SkillResult(
            status=SkillStatus.RUNNING,
            message=f"Executing ACT... {elapsed:.1f}s / {max_duration}s"
        )
    
    def _stop_inference(self):
        """停止推理"""
        if self._start_client:
            try:
                from std_srvs.srv import SetBool
                request = SetBool.Request()
                request.data = False
                self._start_client.call_async(request)
            except Exception as e:
                self.log_error(f"Failed to stop inference: {e}")
    
    def halt(self):
        """中断执行"""
        self.log_info(f"[ACTExecute] Halting")
        self._stop_inference()
        self._is_executing = False
    
    def cleanup(self):
        """清理资源"""
        self._stop_inference()
        self._is_executing = False
        self._policy = None
        self._executor = None


class ACTLoadModelNode(SkillNode):
    """
    ACT 模型加载节点
    
    预先加载模型，避免执行时的加载延迟
    
    参数:
        action_id: 动作 ID（从 model_actions 加载）
        model_name: 模型名称
        model_path: 模型完整路径
    """
    
    NODE_TYPE = "ACTLoadModel"
    
    PARAM_SCHEMA = {
        'action_id': {'type': 'string', 'required': False},  # 动作 ID (优先)
        'model_name': {'type': 'string', 'required': False},
        'model_path': {'type': 'string', 'required': False},
    }
    
    MODEL_ACTIONS_DIR = os.path.expanduser("~/qyh-robot-system/model_actions")
    
    def __init__(self, node_id: str, params: Dict[str, Any] = None, **kwargs):
        super().__init__(node_id, params, **kwargs)
        self._load_client = None
    
    def setup(self) -> bool:
        if not self.ros_node:
            return True
        
        try:
            from std_srvs.srv import Trigger
            self._load_client = self.ros_node.create_client(
                Trigger,
                '/act_inference_node/load_model'
            )
            return True
        except Exception as e:
            return False
    
    def execute(self) -> SkillResult:
        if not self._load_client:
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message="Model loaded (mock)"
            )
        
        from std_srvs.srv import Trigger
        
        if not self._load_client.wait_for_service(timeout_sec=2.0):
            return SkillResult(
                status=SkillStatus.FAILURE,
                message="Load model service not available"
            )
        
        request = Trigger.Request()
        future = self._load_client.call_async(request)
        
        while not future.done():
            time.sleep(0.01)
        
        result = future.result()
        
        if result.success:
            return SkillResult(
                status=SkillStatus.SUCCESS,
                message=result.message
            )
        else:
            return SkillResult(
                status=SkillStatus.FAILURE,
                message=result.message
            )


# 技能注册表（供任务引擎使用）
ACT_SKILL_REGISTRY = {
    'ACTExecute': ACTExecuteNode,
    'ACTLoadModel': ACTLoadModelNode,
}
