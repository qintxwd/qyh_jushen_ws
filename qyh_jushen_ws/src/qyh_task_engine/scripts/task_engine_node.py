#!/usr/bin/env python3
"""
任务引擎 ROS2 节点

提供任务执行服务和状态发布
"""

import json
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup

from qyh_task_engine_msgs.msg import TaskStatus, NodeStatus
from qyh_task_engine_msgs.srv import (
    ExecuteTask, PauseTask, ResumeTask, CancelTask, GetTaskStatus
)
from qyh_task_engine_msgs.action import ExecuteTask as ExecuteTaskAction

from qyh_task_engine import BehaviorTreeEngine, TaskParser


class TaskEngineNode(Node):
    """任务引擎 ROS2 节点"""
    
    def __init__(self):
        super().__init__('task_engine_node')
        
        # 参数
        self.declare_parameter('tick_rate', 10.0)
        self.declare_parameter('status_publish_rate', 5.0)
        
        tick_rate = self.get_parameter('tick_rate').value
        status_rate = self.get_parameter('status_publish_rate').value
        
        # 创建行为树引擎
        self.engine = BehaviorTreeEngine(
            ros_node=self,
            tick_rate=tick_rate,
            on_status_update=self._on_status_update
        )
        
        # 回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 服务
        self.execute_srv = self.create_service(
            ExecuteTask, '/task_engine/execute',
            self._handle_execute,
            callback_group=self.callback_group
        )
        
        self.pause_srv = self.create_service(
            PauseTask, '/task_engine/pause',
            self._handle_pause,
            callback_group=self.callback_group
        )
        
        self.resume_srv = self.create_service(
            ResumeTask, '/task_engine/resume',
            self._handle_resume,
            callback_group=self.callback_group
        )
        
        self.cancel_srv = self.create_service(
            CancelTask, '/task_engine/cancel',
            self._handle_cancel,
            callback_group=self.callback_group
        )
        
        self.status_srv = self.create_service(
            GetTaskStatus, '/task_engine/status',
            self._handle_get_status,
            callback_group=self.callback_group
        )
        
        # Action Server
        self._action_server = ActionServer(
            self,
            ExecuteTaskAction,
            '/task_engine/execute_action',
            self._execute_action_callback,
            callback_group=self.callback_group
        )
        
        # 状态发布器
        self.status_pub = self.create_publisher(
            TaskStatus, '/task_engine/status', 10
        )
        
        self.node_status_pub = self.create_publisher(
            NodeStatus, '/task_engine/node_status', 10
        )
        
        # 定时发布状态
        self.status_timer = self.create_timer(
            1.0 / status_rate,
            self._publish_status
        )
        
        self.get_logger().info('Task Engine Node started')
        self.get_logger().info(f'  Tick rate: {tick_rate} Hz')
        self.get_logger().info(f'  Status rate: {status_rate} Hz')
    
    def _handle_execute(self, request, response):
        """处理执行任务请求"""
        self.get_logger().info('Received execute task request')
        
        # 加载任务
        success, task_id, message = self.engine.load_task(request.task_json)
        if not success:
            response.success = False
            response.task_id = ''
            response.message = message
            return response
        
        # 开始执行
        success, message = self.engine.start()
        response.success = success
        response.task_id = task_id if success else ''
        response.message = message
        
        return response
    
    def _handle_pause(self, request, response):
        """处理暂停任务请求"""
        success, message = self.engine.pause()
        response.success = success
        response.message = message
        return response
    
    def _handle_resume(self, request, response):
        """处理恢复任务请求"""
        success, message = self.engine.resume()
        response.success = success
        response.message = message
        return response
    
    def _handle_cancel(self, request, response):
        """处理取消任务请求"""
        success, message = self.engine.cancel()
        response.success = success
        response.message = message
        return response
    
    def _handle_get_status(self, request, response):
        """处理获取状态请求"""
        status_dict = self.engine.get_status()
        
        response.success = True
        response.message = ''
        response.status = self._dict_to_task_status(status_dict)
        
        return response
    
    def _execute_action_callback(self, goal_handle):
        """Action 执行回调"""
        self.get_logger().info('Received execute action goal')
        
        request = goal_handle.request
        
        # 加载任务
        success, task_id, message = self.engine.load_task(request.task_json)
        if not success:
            goal_handle.abort()
            result = ExecuteTaskAction.Result()
            result.success = False
            result.task_id = ''
            result.final_status = 'failure'
            result.message = message
            return result
        
        # 开始执行
        success, message = self.engine.start()
        if not success:
            goal_handle.abort()
            result = ExecuteTaskAction.Result()
            result.success = False
            result.task_id = task_id
            result.final_status = 'failure'
            result.message = message
            return result
        
        # 等待完成，同时发送反馈
        import time
        while self.engine.is_running():
            if goal_handle.is_cancel_requested:
                self.engine.cancel()
                goal_handle.canceled()
                result = ExecuteTaskAction.Result()
                result.success = False
                result.task_id = task_id
                result.final_status = 'cancelled'
                result.message = 'Task cancelled by request'
                return result
            
            # 发送反馈
            feedback = ExecuteTaskAction.Feedback()
            status_dict = self.engine.get_status()
            feedback.task_status = self._dict_to_task_status(status_dict)
            feedback.node_statuses = self._dict_to_node_statuses(status_dict)
            goal_handle.publish_feedback(feedback)
            
            time.sleep(0.1)
        
        # 完成
        goal_handle.succeed()
        
        final_status = self.engine.get_status()
        result = ExecuteTaskAction.Result()
        result.success = final_status['status'] == 'success'
        result.task_id = task_id
        result.final_status = final_status['status']
        result.message = f"Task {final_status['status']}"
        result.total_time = final_status['elapsed_time']
        
        return result
    
    def _on_status_update(self, status_dict: dict):
        """状态更新回调（从引擎调用）"""
        # 发布到 Topic
        self.status_pub.publish(self._dict_to_task_status(status_dict))
    
    def _publish_status(self):
        """定时发布状态"""
        if self.engine.is_running():
            status_dict = self.engine.get_status()
            self.status_pub.publish(self._dict_to_task_status(status_dict))
    
    def _dict_to_task_status(self, status_dict: dict) -> TaskStatus:
        """将字典转换为 TaskStatus 消息"""
        msg = TaskStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.task_id = status_dict.get('task_id', '') or ''
        msg.task_name = status_dict.get('task_name', '') or ''
        msg.status = status_dict.get('status', 'idle')
        msg.current_node_id = status_dict.get('current_node_id', '') or ''
        msg.completed_nodes = status_dict.get('completed_nodes', 0)
        msg.total_nodes = status_dict.get('total_nodes', 0)
        msg.progress = float(status_dict.get('progress', 0.0))
        msg.message = status_dict.get('message', '')
        msg.elapsed_time = float(status_dict.get('elapsed_time', 0.0))
        return msg
    
    def _dict_to_node_statuses(self, status_dict: dict) -> list:
        """将字典转换为 NodeStatus 消息列表"""
        node_statuses = []
        for node_state in status_dict.get('node_statuses', []):
            msg = NodeStatus()
            msg.node_id = node_state.get('node_id', '')
            msg.node_type = node_state.get('node_type', '')
            msg.node_name = node_state.get('node_name', '')
            msg.status = node_state.get('status', 'idle')
            msg.message = node_state.get('result', {}).get('message', '') if node_state.get('result') else ''
            msg.duration = float(node_state.get('duration', 0.0))
            node_statuses.append(msg)
        return node_statuses


def main(args=None):
    rclpy.init(args=args)
    node = TaskEngineNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
