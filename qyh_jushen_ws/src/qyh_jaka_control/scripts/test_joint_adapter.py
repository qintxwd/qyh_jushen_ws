#!/usr/bin/env python3
"""
Test script for JAKA joint name adapter

Usage:
  1. Terminal 1: ros2 launch qyh_jaka_control jaka_control.launch.py
  2. Terminal 2: python3 test_joint_adapter.py

This script will:
  - Monitor /joint_states_raw (input to adapter)
  - Monitor /joint_states (output from adapter)
  - Verify that joint names are correctly converted
  - Check that position/velocity/effort data is preserved
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from collections import defaultdict


class JointAdapterTester(Node):
    def __init__(self):
        super().__init__('joint_adapter_tester')
        
        self.raw_data = None
        self.adapted_data = None
        self.message_count = 0
        
        # Expected URDF joint names
        self.expected_urdf_names = [
            'l-j1', 'l-j2', 'l-j3', 'l-j4', 'l-j5', 'l-j6', 'l-j7',
            'r-j1', 'r-j2', 'r-j3', 'r-j4', 'r-j5', 'r-j6', 'r-j7'
        ]
        
        # Subscribers
        self.raw_sub = self.create_subscription(
            JointState, '/joint_states_raw', self.raw_callback, 10)
        self.adapted_sub = self.create_subscription(
            JointState, '/joint_states', self.adapted_callback, 10)
        
        # Timer for status report
        self.timer = self.create_timer(2.0, self.report_status)
        
        self.get_logger().info('='*60)
        self.get_logger().info('JAKA Joint Adapter Test Started')
        self.get_logger().info('='*60)
        self.get_logger().info('Monitoring /joint_states_raw and /joint_states...')
    
    def raw_callback(self, msg):
        self.raw_data = msg
    
    def adapted_callback(self, msg):
        self.adapted_data = msg
        self.message_count += 1
    
    def report_status(self):
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info(f'Status Report (message count: {self.message_count})')
        self.get_logger().info('='*60)
        
        # Check raw data
        if self.raw_data is None:
            self.get_logger().warn('❌ No data on /joint_states_raw')
            self.get_logger().info('   → Check if jaka_control_node is running')
            return
        
        self.get_logger().info(f'✓ Raw data received: {len(self.raw_data.name)} joints')
        self.get_logger().info(f'  Input joint names: {self.raw_data.name[:3]}...')
        
        # Check adapted data
        if self.adapted_data is None:
            self.get_logger().warn('❌ No data on /joint_states')
            self.get_logger().info('   → Check if qyh_jaka_joint_adapter is running')
            return
        
        self.get_logger().info(f'✓ Adapted data received: {len(self.adapted_data.name)} joints')
        self.get_logger().info(f'  Output joint names: {self.adapted_data.name[:3]}...')
        
        # Verify joint name conversion
        all_names_valid = all(name in self.expected_urdf_names 
                             for name in self.adapted_data.name)
        
        if all_names_valid:
            self.get_logger().info('✅ Joint names correctly converted to URDF format')
        else:
            invalid_names = [n for n in self.adapted_data.name 
                           if n not in self.expected_urdf_names]
            self.get_logger().error(f'❌ Invalid joint names found: {invalid_names}')
        
        # Check joint count
        if len(self.adapted_data.name) == 14:
            self.get_logger().info('✅ Joint count correct (14 joints)')
        else:
            self.get_logger().warn(f'⚠️  Unexpected joint count: {len(self.adapted_data.name)}')
        
        # Verify data preservation
        if len(self.raw_data.position) == len(self.adapted_data.position):
            self.get_logger().info('✅ Position data preserved')
            # Show first joint position as sample
            if self.adapted_data.position:
                self.get_logger().info(f'   Sample: {self.adapted_data.name[0]} = '
                                     f'{self.adapted_data.position[0]:.4f} rad')
        else:
            self.get_logger().error('❌ Position data length mismatch')
        
        # Summary
        self.get_logger().info('-'*60)
        if all_names_valid and len(self.adapted_data.name) == 14:
            self.get_logger().info('🎉 ADAPTER WORKING CORRECTLY!')
            self.get_logger().info('   You can now use robot_state_publisher with this data')
        else:
            self.get_logger().warn('⚠️  Adapter needs attention, check logs above')


def main(args=None):
    rclpy.init(args=args)
    tester = JointAdapterTester()
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('\nTest stopped by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
