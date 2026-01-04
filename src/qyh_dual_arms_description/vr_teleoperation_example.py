#!/usr/bin/env python3
"""
VR遥操作示例节点
展示如何使用base_vr坐标系进行增量控制
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
import numpy as np

class VRTeleoperationNode(Node):
    """VR遥操作节点"""
    
    def __init__(self):
        super().__init__('vr_teleoperation_node')
        
        # TF2监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅VR手柄数据（假设VR系统发布的数据已经在base_vr坐标系下）
        self.vr_right_sub = self.create_subscription(
            TwistStamped,
            '/vr/right_controller/twist',  # VR右手柄速度增量
            self.right_controller_callback,
            10
        )
        
        self.vr_left_sub = self.create_subscription(
            TwistStamped,
            '/vr/left_controller/twist',  # VR左手柄速度增量
            self.left_controller_callback,
            10
        )
        
        # 发布到机械臂笛卡尔速度控制器
        self.right_arm_pub = self.create_publisher(
            TwistStamped,
            '/right_arm/twist_controller/command',
            10
        )
        
        self.left_arm_pub = self.create_publisher(
            TwistStamped,
            '/left_arm/twist_controller/command',
            10
        )
        
        # 控制参数
        self.scale_factor = 1.0  # 速度缩放因子
        self.max_linear_vel = 0.5  # m/s
        self.max_angular_vel = 1.0  # rad/s
        
        self.get_logger().info('VR Teleoperation Node Started')
        self.get_logger().info('Using base_vr coordinate frame for VR input')
    
    def right_controller_callback(self, msg: TwistStamped):
        """处理右手柄增量控制 -> 右臂(rt)"""
        self.process_controller_input(msg, 'rt', self.right_arm_pub)
    
    def left_controller_callback(self, msg: TwistStamped):
        """处理左手柄增量控制 -> 左臂(lt)"""
        self.process_controller_input(msg, 'lt', self.left_arm_pub)
    
    def process_controller_input(self, msg: TwistStamped, target_frame: str, publisher):
        """
        处理VR手柄输入并转换到机械臂末端坐标系
        
        Args:
            msg: VR手柄的Twist消息（在base_vr坐标系下）
            target_frame: 目标坐标系（'rt'或'lt'）
            publisher: 发布器
        """
        try:
            # 方案A: 如果VR数据已经在base_vr坐标系下，直接使用
            # 然后通过TF查询从base_vr到目标末端(rt/lt)的转换
            transform = self.tf_buffer.lookup_transform(
                target_frame,      # 目标：机械臂末端
                'base_vr',         # 源：VR坐标系
                rclpy.time.Time(), # 最新时间
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # 转换速度向量
            twist_transformed = self.transform_twist(msg.twist, transform)
            
            # 应用缩放和限制
            twist_scaled = self.apply_scaling_and_limits(twist_transformed)
            
            # 发布到机械臂控制器
            output_msg = TwistStamped()
            output_msg.header.stamp = self.get_clock().now().to_msg()
            output_msg.header.frame_id = target_frame
            output_msg.twist = twist_scaled
            
            publisher.publish(output_msg)
            
        except Exception as e:
            self.get_logger().warn(f'Transform failed for {target_frame}: {e}')
    
    def transform_twist(self, twist, transform):
        """
        使用TF变换Twist消息
        
        Args:
            twist: 原始Twist
            transform: TF变换
        
        Returns:
            变换后的Twist
        """
        from geometry_msgs.msg import Twist, Vector3
        from scipy.spatial.transform import Rotation as R
        
        # 提取旋转
        q = transform.transform.rotation
        rotation = R.from_quat([q.x, q.y, q.z, q.w])
        rot_matrix = rotation.as_matrix()
        
        # 转换线速度
        linear_vec = np.array([twist.linear.x, twist.linear.y, twist.linear.z])
        linear_transformed = rot_matrix @ linear_vec
        
        # 转换角速度
        angular_vec = np.array([twist.angular.x, twist.angular.y, twist.angular.z])
        angular_transformed = rot_matrix @ angular_vec
        
        # 构造输出
        result = Twist()
        result.linear.x = float(linear_transformed[0])
        result.linear.y = float(linear_transformed[1])
        result.linear.z = float(linear_transformed[2])
        result.angular.x = float(angular_transformed[0])
        result.angular.y = float(angular_transformed[1])
        result.angular.z = float(angular_transformed[2])
        
        return result
    
    def apply_scaling_and_limits(self, twist):
        """应用缩放因子和速度限制"""
        from geometry_msgs.msg import Twist
        import math
        
        result = Twist()
        
        # 应用缩放
        linear_x = twist.linear.x * self.scale_factor
        linear_y = twist.linear.y * self.scale_factor
        linear_z = twist.linear.z * self.scale_factor
        
        # 限制线速度
        linear_mag = math.sqrt(linear_x**2 + linear_y**2 + linear_z**2)
        if linear_mag > self.max_linear_vel:
            scale = self.max_linear_vel / linear_mag
            linear_x *= scale
            linear_y *= scale
            linear_z *= scale
        
        result.linear.x = linear_x
        result.linear.y = linear_y
        result.linear.z = linear_z
        
        # 应用缩放和限制角速度
        angular_x = twist.angular.x * self.scale_factor
        angular_y = twist.angular.y * self.scale_factor
        angular_z = twist.angular.z * self.scale_factor
        
        angular_mag = math.sqrt(angular_x**2 + angular_y**2 + angular_z**2)
        if angular_mag > self.max_angular_vel:
            scale = self.max_angular_vel / angular_mag
            angular_x *= scale
            angular_y *= scale
            angular_z *= scale
        
        result.angular.x = angular_x
        result.angular.y = angular_y
        result.angular.z = angular_z
        
        return result


def main(args=None):
    rclpy.init(args=args)
    node = VRTeleoperationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
