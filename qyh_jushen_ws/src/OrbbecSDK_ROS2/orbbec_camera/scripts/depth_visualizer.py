#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthVisualizer(Node):
    def __init__(self):
        super().__init__('depth_visualizer')
        self.subscription = self.create_subscription(
            Image,
            '/head_camera/depth/image_raw',
            self.listener_callback,
            qos_profile_sensor_data)
        self.publisher = self.create_publisher(Image, '/head_camera/depth/image_visual', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Depth Visualizer Node Started')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Normalize the 16-bit depth image to 8-bit for visualization
            # Adjust the alpha (scale) to make the depth map visible
            # Assuming max depth range of ~5000mm (5m) for better contrast
            # You can adjust 5000.0 to match your specific environment range
            depth_normalized = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            
            # Apply a colormap (JET is common for depth)
            depth_colored = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            
            # Convert OpenCV image back to ROS Image message
            visual_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding="bgr8")
            visual_msg.header = msg.header
            
            self.publisher.publish(visual_msg)
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    depth_visualizer = DepthVisualizer()
    rclpy.spin(depth_visualizer)
    depth_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
