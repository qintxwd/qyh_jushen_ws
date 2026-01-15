#!/usr/bin/env python3
"""
测试任务脚本

用于测试任务引擎的基本功能
"""

import rclpy
from rclpy.node import Node
import json

from qyh_task_engine_msgs.srv import ExecuteTask, GetTaskStatus


# 测试任务 JSON
TEST_TASK_SIMPLE = {
    "task_id": "test_001",
    "name": "Simple Test Task",
    "root": {
        "type": "Sequence",
        "children": [
            {
                "type": "Wait",
                "name": "等待1秒",
                "params": {"duration": 1.0}
            },
            {
                "type": "HeadLookAt",
                "name": "头部转向",
                "params": {"pitch": 0.2, "yaw": -0.3}
            },
            {
                "type": "Wait",
                "name": "等待2秒",
                "params": {"duration": 2.0}
            }
        ]
    }
}

TEST_TASK_ARM = {
    "task_id": "test_002",
    "name": "Arm Movement Test",
    "root": {
        "type": "Sequence",
        "children": [
            {
                "type": "ArmMoveJ",
                "name": "左臂移动到观察位",
                "params": {
                    "side": "left",
                    "pose_name": "observe_pose"
                }
            },
            {
                "type": "Wait",
                "params": {"duration": 1.0}
            },
            {
                "type": "ArmMoveJ",
                "name": "左臂回到初始位",
                "params": {
                    "side": "left",
                    "pose_name": "home_pose"
                }
            }
        ]
    }
}

TEST_TASK_PARALLEL = {
    "task_id": "test_003",
    "name": "Parallel Test Task",
    "root": {
        "type": "Sequence",
        "children": [
            {
                "type": "Parallel",
                "name": "双臂同时移动",
                "children": [
                    {
                        "type": "ArmMoveJ",
                        "name": "左臂",
                        "params": {"side": "left", "pose_name": "observe_pose"}
                    },
                    {
                        "type": "ArmMoveJ",
                        "name": "右臂",
                        "params": {"side": "right", "pose_name": "observe_pose"}
                    }
                ]
            },
            {
                "type": "Wait",
                "params": {"duration": 1.0}
            }
        ]
    }
}


class TestTaskClient(Node):
    """测试任务客户端"""
    
    def __init__(self):
        super().__init__('test_task_client')
        
        self.execute_client = self.create_client(
            ExecuteTask, '/task_engine/execute'
        )
        self.status_client = self.create_client(
            GetTaskStatus, '/task_engine/status'
        )
        
        self.get_logger().info('Test Task Client started')
    
    def execute_task(self, task_dict: dict):
        """执行任务"""
        while not self.execute_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for task engine service...')
        
        request = ExecuteTask.Request()
        request.task_json = json.dumps(task_dict)
        
        self.get_logger().info(f'Executing task: {task_dict["name"]}')
        
        future = self.execute_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f'Task started: {response.task_id}')
        else:
            self.get_logger().error(f'Failed to start task: {response.message}')
        
        return response
    
    def get_status(self):
        """获取任务状态"""
        while not self.status_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for status service...')
        
        request = GetTaskStatus.Request()
        future = self.status_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            status = response.status
            self.get_logger().info(
                f'Task: {status.task_name} ({status.task_id})\n'
                f'  Status: {status.status}\n'
                f'  Progress: {status.progress * 100:.1f}%\n'
                f'  Current Node: {status.current_node_id}\n'
                f'  Elapsed: {status.elapsed_time:.1f}s'
            )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    
    client = TestTaskClient()
    
    # 选择测试任务
    import sys
    if len(sys.argv) > 1:
        test_name = sys.argv[1]
        if test_name == 'arm':
            task = TEST_TASK_ARM
        elif test_name == 'parallel':
            task = TEST_TASK_PARALLEL
        else:
            task = TEST_TASK_SIMPLE
    else:
        task = TEST_TASK_SIMPLE
    
    # 执行任务
    response = client.execute_task(task)
    
    if response.success:
        # 等待一会儿再检查状态
        import time
        time.sleep(1.0)
        client.get_status()
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
