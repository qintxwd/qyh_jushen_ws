#!/usr/bin/env python3
"""
ACT 推理测试脚本

用于测试 ACT 推理节点是否正常工作

使用方法:
    ros2 run qyh_act_inference test_act_inference.py
"""

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String


class ACTInferenceTestNode(Node):
    """ACT 推理测试节点"""
    
    def __init__(self):
        super().__init__('act_inference_test')
        
        self.get_logger().info("="*50)
        self.get_logger().info("ACT Inference Test Node")
        self.get_logger().info("="*50)
        
        # 创建服务客户端
        self._load_model_client = self.create_client(
            Trigger,
            '/act_inference_node/load_model'
        )
        
        self._start_client = self.create_client(
            SetBool,
            '/act_inference_node/start'
        )
        
        self._reset_client = self.create_client(
            Trigger,
            '/act_inference_node/reset'
        )
        
        # 状态订阅
        self._status_sub = self.create_subscription(
            String,
            '/act_inference_node/status',
            self._status_callback,
            10
        )
        
        self._current_status = "unknown"
    
    def _status_callback(self, msg):
        self._current_status = msg.data
        self.get_logger().info(f"Status: {self._current_status}")
    
    def wait_for_services(self, timeout=5.0):
        """等待服务可用"""
        self.get_logger().info("Waiting for services...")
        
        services = [
            ('/act_inference_node/load_model', self._load_model_client),
            ('/act_inference_node/start', self._start_client),
        ]
        
        for name, client in services:
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f"Service {name} not available!")
                return False
            self.get_logger().info(f"  {name} - OK")
        
        return True
    
    def load_model(self):
        """加载模型"""
        self.get_logger().info("Loading model...")
        
        request = Trigger.Request()
        future = self._load_model_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.done():
            result = future.result()
            if result.success:
                self.get_logger().info(f"Model loaded: {result.message}")
                return True
            else:
                self.get_logger().error(f"Failed to load model: {result.message}")
                return False
        else:
            self.get_logger().error("Load model timeout")
            return False
    
    def start_inference(self):
        """启动推理"""
        self.get_logger().info("Starting inference...")
        
        request = SetBool.Request()
        request.data = True
        future = self._start_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            result = future.result()
            if result.success:
                self.get_logger().info(f"Inference started: {result.message}")
                return True
            else:
                self.get_logger().error(f"Failed to start: {result.message}")
                return False
        return False
    
    def stop_inference(self):
        """停止推理"""
        self.get_logger().info("Stopping inference...")
        
        request = SetBool.Request()
        request.data = False
        future = self._start_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            result = future.result()
            self.get_logger().info(f"Inference stopped: {result.message}")
            return result.success
        return False
    
    def run_test(self, duration=10.0):
        """运行测试"""
        self.get_logger().info(f"Running test for {duration}s...")
        
        # 1. 等待服务
        if not self.wait_for_services():
            return False
        
        # 2. 加载模型
        if not self.load_model():
            return False
        
        # 3. 启动推理
        if not self.start_inference():
            return False
        
        # 4. 运行一段时间
        self.get_logger().info(f"Running for {duration}s...")
        start_time = time.time()
        
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        # 5. 停止推理
        self.stop_inference()
        
        self.get_logger().info("Test completed!")
        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = ACTInferenceTestNode()
    
    try:
        node.run_test(duration=10.0)
    except KeyboardInterrupt:
        node.stop_inference()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
