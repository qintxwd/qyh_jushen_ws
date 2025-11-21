#!/usr/bin/env python3
"""
键盘控制节点 - 用于双臂机械臂示教
支持直接关节角度控制和录制触发

按键映射：
  数字键 1-7: 选择要控制的关节
  Q/A: 增加/减少选中关节角度
  空格: 切换左右臂
  R: 开始/停止录制
  H: 显示帮助
  ESC: 退出
"""

import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool
import threading


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # 发布者
        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            10
        )
        self.record_trigger_pub = self.create_publisher(
            Bool,
            '/recording_trigger',
            10
        )
        
        # 订阅关节状态
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        # 状态变量
        self.current_arm = 'right'  # 'right' or 'left'
        self.selected_joint = 0  # 0-6
        self.joint_step = 0.1  # 每次移动的角度（弧度）
        self.is_recording = False
        
        # 当前关节位置
        self.right_joint_positions = [0.0] * 7
        self.left_joint_positions = [0.0] * 7
        
        # 关节名称
        self.right_joint_names = ['r-j1', 'r-j2', 'r-j3', 'r-j4', 'r-j5', 'r-j6', 'r-j7']
        self.left_joint_names = ['l-j1', 'l-j2', 'l-j3', 'l-j4', 'l-j5', 'l-j6', 'l-j7']
        
        self.get_logger().info('Teleop Keyboard Node initialized')
        self.print_help()
    
    def joint_states_callback(self, msg):
        """更新当前关节位置"""
        for i, name in enumerate(msg.name):
            if name in self.right_joint_names:
                idx = self.right_joint_names.index(name)
                self.right_joint_positions[idx] = msg.position[i]
            elif name in self.left_joint_names:
                idx = self.left_joint_names.index(name)
                self.left_joint_positions[idx] = msg.position[i]
    
    def print_help(self):
        """打印帮助信息"""
        help_text = """
╔══════════════════════════════════════════════════════════╗
║           双臂机械臂键盘控制                              ║
╚══════════════════════════════════════════════════════════╝

控制说明：
  数字键 1-7 : 选择要控制的关节 (J1-J7)
  Q / A      : 增加 / 减少 选中关节角度 (步长: 0.1 rad)
  空格       : 切换左右臂
  R          : 开始/停止录制
  H          : 显示此帮助
  ESC        : 退出程序

当前状态：
  控制臂: {}
  选中关节: J{}
  录制状态: {}
        """.format(
            self.current_arm.upper(),
            self.selected_joint + 1,
            "进行中" if self.is_recording else "停止"
        )
        print(help_text)
    
    def print_status(self):
        """打印当前状态"""
        positions = self.right_joint_positions if self.current_arm == 'right' else self.left_joint_positions
        
        status = "\n控制臂: {} | 选中关节: J{} | 录制: {}\n".format(
            self.current_arm.upper(),
            self.selected_joint + 1,
            "ON" if self.is_recording else "OFF"
        )
        status += "当前关节位置: "
        for i, pos in enumerate(positions):
            marker = " >>>" if i == self.selected_joint else ""
            status += "J{}:{:.2f}{} ".format(i+1, pos, marker)
        
        print(status)
    
    def send_joint_command(self, joint_idx, delta):
        """发送关节控制命令"""
        if self.current_arm == 'right':
            positions = self.right_joint_positions.copy()
            joint_names = self.right_joint_names
            publisher = self.right_arm_pub
        else:
            positions = self.left_joint_positions.copy()
            joint_names = self.left_joint_names
            publisher = self.left_arm_pub
        
        # 更新目标位置
        positions[joint_idx] += delta
        
        # 限制范围 (-π to π)
        positions[joint_idx] = max(-3.14, min(3.14, positions[joint_idx]))
        
        # 创建轨迹消息
        traj = JointTrajectory()
        traj.joint_names = joint_names
        
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=500000000)  # 0.5秒
        
        traj.points = [point]
        
        # 发布
        publisher.publish(traj)
        
        self.get_logger().info(
            f'{self.current_arm.upper()} arm J{joint_idx+1}: {positions[joint_idx]:.3f}'
        )
    
    def toggle_recording(self):
        """切换录制状态"""
        self.is_recording = not self.is_recording
        msg = Bool()
        msg.data = self.is_recording
        self.record_trigger_pub.publish(msg)
        
        if self.is_recording:
            self.get_logger().info('🔴 录制开始')
            print("\n🔴 录制开始！执行你的演示动作...")
        else:
            self.get_logger().info('⏹️  录制停止')
            print("\n⏹️  录制已停止")
    
    def handle_key(self, key):
        """处理键盘输入"""
        if key == ' ':  # 空格 - 切换臂
            self.current_arm = 'left' if self.current_arm == 'right' else 'right'
            self.get_logger().info(f'切换到 {self.current_arm.upper()} 臂')
            self.print_status()
        
        elif key in '1234567':  # 数字键 - 选择关节
            self.selected_joint = int(key) - 1
            self.get_logger().info(f'选择关节 J{self.selected_joint + 1}')
            self.print_status()
        
        elif key.lower() == 'q':  # Q - 增加角度
            self.send_joint_command(self.selected_joint, self.joint_step)
            self.print_status()
        
        elif key.lower() == 'a':  # A - 减少角度
            self.send_joint_command(self.selected_joint, -self.joint_step)
            self.print_status()
        
        elif key.lower() == 'r':  # R - 切换录制
            self.toggle_recording()
        
        elif key.lower() == 'h':  # H - 帮助
            self.print_help()
            self.print_status()
        
        elif key == '\x1b':  # ESC - 退出
            self.get_logger().info('退出程序')
            return False
        
        return True


def get_key():
    """获取键盘输入（非阻塞）"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def spin_node(node):
    """在单独线程中运行ROS节点"""
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    # 在单独线程中运行ROS节点
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()
    
    print("\n等待关节状态信息...")
    import time
    time.sleep(2)  # 等待接收初始关节状态
    
    node.print_status()
    print("\n准备就绪！开始控制...")
    
    try:
        while True:
            key = get_key()
            if not node.handle_key(key):
                break
    
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
