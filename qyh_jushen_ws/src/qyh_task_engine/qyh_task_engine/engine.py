"""
行为树引擎 (Behavior Tree Engine)

负责行为树的执行、状态管理和实时反馈
"""

import time
import threading
import uuid
from typing import Dict, Any, Optional, List, Callable, Union
from enum import Enum

from .base_node import SkillNode, SkillStatus
from .composite_nodes import CompositeNode
from .parser import TaskParser


class TaskState(Enum):
    """任务状态"""
    IDLE = "idle"           # 空闲
    RUNNING = "running"     # 执行中
    PAUSED = "paused"       # 已暂停
    SUCCESS = "success"     # 成功完成
    FAILURE = "failure"     # 执行失败
    CANCELLED = "cancelled" # 已取消


class BehaviorTreeEngine:
    """
    行为树执行引擎
    
    负责:
    - 解析任务 JSON
    - 执行行为树 (Tick 循环)
    - 管理任务状态 (暂停/恢复/取消)
    - 发送状态更新回调
    """
    
    def __init__(
        self,
        ros_node = None,
        tick_rate: float = 10.0,
        on_status_update: Callable[[Dict[str, Any]], None] = None
    ):
        """
        初始化引擎
        
        Args:
            ros_node: ROS2 节点引用
            tick_rate: Tick 频率 (Hz)
            on_status_update: 状态更新回调函数
        """
        self.ros_node = ros_node
        self.tick_rate = tick_rate
        self.tick_period = 1.0 / tick_rate
        self.on_status_update = on_status_update
        
        self.parser = TaskParser(ros_node)
        
        # 当前任务状态
        self._task_id: Optional[str] = None
        self._task_name: Optional[str] = None
        self._root_node: Optional[Union[SkillNode, CompositeNode]] = None
        self._task_state: TaskState = TaskState.IDLE
        self._start_time: Optional[float] = None
        
        # 执行控制
        self._running = False
        self._paused = False
        self._cancel_requested = False
        self._exec_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
    
    def load_task(self, task_json: Union[str, dict]) -> tuple:
        """
        加载任务
        
        Args:
            task_json: 任务 JSON
        
        Returns:
            (success, task_id, message)
        """
        with self._lock:
            if self._task_state == TaskState.RUNNING:
                return False, None, "Another task is running"
            
            try:
                # 验证 JSON
                is_valid, error = self.parser.validate(task_json)
                if not is_valid:
                    return False, None, f"Invalid task: {error}"
                
                # 解析数据
                if isinstance(task_json, str):
                    task_data = __import__('json').loads(task_json)
                else:
                    task_data = task_json
                
                # 解析行为树
                self._root_node = self.parser.parse(task_json)
                
                # 生成任务 ID
                self._task_id = task_data.get('task_id', str(uuid.uuid4())[:8])
                self._task_name = task_data.get('name', 'Unnamed Task')
                self._task_state = TaskState.IDLE
                self._start_time = None
                
                self._log_info(f"Task loaded: {self._task_name} ({self._task_id})")
                
                return True, self._task_id, "Task loaded successfully"
            
            except Exception as e:
                return False, None, f"Failed to load task: {e}"
    
    def start(self) -> tuple:
        """
        开始执行任务
        
        Returns:
            (success, message)
        """
        with self._lock:
            if self._root_node is None:
                return False, "No task loaded"
            
            if self._task_state == TaskState.RUNNING:
                return False, "Task already running"
            
            self._running = True
            self._paused = False
            self._cancel_requested = False
            self._task_state = TaskState.RUNNING
            self._start_time = time.time()
            
            # 启动执行线程
            self._exec_thread = threading.Thread(target=self._execution_loop, daemon=True)
            self._exec_thread.start()
            
            self._log_info(f"Task started: {self._task_name}")
            return True, "Task started"
    
    def pause(self) -> tuple:
        """暂停任务"""
        with self._lock:
            if self._task_state != TaskState.RUNNING:
                return False, "Task not running"
            
            self._paused = True
            self._task_state = TaskState.PAUSED
            self._log_info("Task paused")
            return True, "Task paused"
    
    def resume(self) -> tuple:
        """恢复任务"""
        with self._lock:
            if self._task_state != TaskState.PAUSED:
                return False, "Task not paused"
            
            self._paused = False
            self._task_state = TaskState.RUNNING
            self._log_info("Task resumed")
            return True, "Task resumed"
    
    def cancel(self) -> tuple:
        """取消任务"""
        with self._lock:
            if self._task_state not in [TaskState.RUNNING, TaskState.PAUSED]:
                return False, "No active task to cancel"
            
            self._cancel_requested = True
            self._running = False
            
            # 中断所有节点
            if self._root_node:
                self._root_node.halt()
            
            self._task_state = TaskState.CANCELLED
            self._log_info("Task cancelled")
            return True, "Task cancelled"
    
    def get_status(self) -> Dict[str, Any]:
        """获取当前任务状态"""
        with self._lock:
            elapsed = 0.0
            if self._start_time:
                elapsed = time.time() - self._start_time
            
            # 收集所有节点状态
            node_states = []
            if self._root_node:
                if isinstance(self._root_node, CompositeNode):
                    node_states = self._root_node.get_all_states()
                else:
                    node_states = [self._root_node.get_state()]
            
            # 计算进度
            completed = sum(1 for s in node_states if s['status'] in ['success', 'failure'])
            total = len(node_states)
            progress = completed / total if total > 0 else 0.0
            
            # 当前节点
            current_node_id = None
            for state in node_states:
                if state['status'] == 'running':
                    current_node_id = state['node_id']
                    break
            
            return {
                'task_id': self._task_id,
                'task_name': self._task_name,
                'status': self._task_state.value,
                'current_node_id': current_node_id,
                'completed_nodes': completed,
                'total_nodes': total,
                'progress': progress,
                'elapsed_time': elapsed,
                'node_statuses': node_states
            }
    
    def _execution_loop(self):
        """执行循环（在独立线程中运行）"""
        self._log_info("Execution loop started")
        
        try:
            while self._running and not self._cancel_requested:
                # 暂停检查
                if self._paused:
                    time.sleep(0.1)
                    continue
                
                # Tick
                tick_start = time.time()
                
                with self._lock:
                    if self._root_node:
                        status = self._root_node.tick()
                        
                        # 发送状态更新
                        if self.on_status_update:
                            self.on_status_update(self.get_status())
                        
                        # 检查是否完成
                        if status == SkillStatus.SUCCESS:
                            self._task_state = TaskState.SUCCESS
                            self._running = False
                            self._log_info("Task completed successfully")
                            break
                        
                        elif status == SkillStatus.FAILURE:
                            self._task_state = TaskState.FAILURE
                            self._running = False
                            self._log_info("Task failed")
                            break
                        
                        elif status == SkillStatus.HALTED:
                            self._task_state = TaskState.CANCELLED
                            self._running = False
                            self._log_info("Task halted")
                            break
                
                # 控制 Tick 频率
                tick_duration = time.time() - tick_start
                sleep_time = self.tick_period - tick_duration
                if sleep_time > 0:
                    time.sleep(sleep_time)
        
        except Exception as e:
            self._log_error(f"Execution error: {e}")
            with self._lock:
                self._task_state = TaskState.FAILURE
                self._running = False
        
        finally:
            # 最终状态更新
            if self.on_status_update:
                self.on_status_update(self.get_status())
            self._log_info("Execution loop ended")
    
    def _log_info(self, msg: str):
        if self.ros_node:
            self.ros_node.get_logger().info(f"[TaskEngine] {msg}")
        else:
            print(f"[INFO] [TaskEngine] {msg}")
    
    def _log_error(self, msg: str):
        if self.ros_node:
            self.ros_node.get_logger().error(f"[TaskEngine] {msg}")
        else:
            print(f"[ERROR] [TaskEngine] {msg}")
    
    def is_running(self) -> bool:
        """是否有任务正在运行"""
        return self._task_state in [TaskState.RUNNING, TaskState.PAUSED]
    
    def get_current_task_id(self) -> Optional[str]:
        """获取当前任务 ID"""
        return self._task_id
