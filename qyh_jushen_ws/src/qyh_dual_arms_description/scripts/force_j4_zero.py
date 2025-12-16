#!/usr/bin/python3
"""
Force j4 joints (l-j4/r-j4) to exactly zero in /joint_states
Subscribes to /joint_states, modifies l-j4 and r-j4 to 0.0, and republishes to /joint_states_corrected
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class ForceJ4Zero(Node):
    def __init__(self):
        super().__init__('force_j4_zero')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            JointState,
            '/joint_states_corrected',
            10
        )
        
        self.get_logger().info('Force j4 zero node started. Listening to /joint_states...')
    
    def joint_state_callback(self, msg):
        """Modify l-j4 and r-j4 to 0.0 and republish"""
        corrected_msg = JointState()
        corrected_msg.header = msg.header
        corrected_msg.name = list(msg.name)
        corrected_msg.position = list(msg.position)
        corrected_msg.velocity = list(msg.velocity) if msg.velocity else []
        corrected_msg.effort = list(msg.effort) if msg.effort else []
        
        # Force l-j4 and r-j4 to zero
        for i, joint_name in enumerate(corrected_msg.name):
            if joint_name in ['l-j4', 'r-j4']:
                if i < len(corrected_msg.position):
                    original_value = corrected_msg.position[i]
                    corrected_msg.position[i] = 0.0
                    self.get_logger().debug(
                        f'Corrected {joint_name}: {original_value:.6f} -> 0.0',
                        throttle_duration_sec=1.0
                    )
        
        self.publisher.publish(corrected_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ForceJ4Zero()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
