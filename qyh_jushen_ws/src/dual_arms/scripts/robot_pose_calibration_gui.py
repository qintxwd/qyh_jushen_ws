#!/usr/bin/env python3
"""
机械臂姿态标定GUI工具

功能:
1. 滑块控制左右机械臂的7个关节
2. 实时显示末端执行器位姿（相对base_link）
3. 保存当前姿态到robot_calibration_poses.yaml

使用方法:
ros2 run dual_arms robot_pose_calibration_gui.py
"""

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
import tkinter as tk
from tkinter import ttk, messagebox
import threading
import yaml
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation as R


class RobotPoseCalibrationGUI(Node):
    """机械臂姿态标定GUI"""
    
    def __init__(self):
        super().__init__('robot_pose_calibration_gui')
        
        # TF监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 关节状态发布（直接发布到joint_states）
        self.joint_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        
        # 定时发布关节状态（持续发布，确保robot_state_publisher能接收）
        self.publish_timer = self.create_timer(0.1, self._publish_joint_states_callback)
        
        # 左右臂末端frame名称
        self.left_ee_frame = 'left_Link7'  # 根据实际URDF调整
        self.right_ee_frame = 'right_Link7'
        self.base_frame = 'base_link'
        
        # 当前关节状态（从GUI滑块设置）
        self.current_joints = {}
        # 初始化所有关节为0
        for i in range(1, 8):
            self.current_joints[f'l-j{i}'] = 0.0
            self.current_joints[f'r-j{i}'] = 0.0
        
        # 配置文件路径
        self.config_file = Path(__file__).parent.parent / 'config' / 'robot_calibration_poses.yaml'
        
        # 定时器：更新TF
        self.create_timer(0.1, self._update_tf)
        
        # 存储TF数据
        self.left_ee_pose = None
        spublish_joint_states_callback(self):
        """定时发布关节状态"""
        if not self.current_joints:
            return
            
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.current_joints.keys())
        msg.position = list(self.current_joints.values())
        self.joint_pub.publish(msg)
        
        # 调试：每2秒打印一次
        self.get_logger().info(
            f'Publishing joint states: {len(msg.name)} joints',
            throttle_duration_sec=2.0)
    
    def _update_tf(self):
        """更新末端位姿"""
        try:
            # 左臂末端
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.left_ee_frame, rclpy.time.Time())
            self.left_ee_pose = self._transform_to_pose(trans)
        except Exception as e:
            pass
        
        try:
            # 右臂末端
            trans = self.tf_buffer.lookup_transform(
                self.base_frame, self.right_ee_frame, rclpy.time.Time())
            self.right_ee_pose = self._transform_to_pose(trans)
        except Exception as e:
            pass
    
    def _transform_to_pose(self, trans):
        """TransformStamped转为位姿字典"""
        return {
            'x': trans.transform.translation.x,
            'y': trans.transform.translation.y,
            'z': trans.transform.translation.z,
            'qx': trans.transform.rotation.x,
            'qy': trans.transform.rotation.y,
            'qz': trans.transform.rotation.z,
            'qw': trans.transform.rotation.w
        }
    
    def update_joint_values(self, joint_values):
        """更新关节值（GUI调用）"""
        self.current_joints.update(joint_values)
    
    def save_pose(self, pose_name, arm_name, pose_data):
        """保存姿态到YAML文件"""
        # 加载现有配置
        if self.config_file.exists():
            with open(self.config_file, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
        else:
            config = {'calibration_poses': {}}
        
        # 更新对应姿态
        if pose_name not in config['calibration_poses']:
            config['calibration_poses'][pose_name] = {}
        
        arm_key = f'{arm_name}_arm'
        config['calibration_poses'][pose_name][arm_key] = {
            'position': {
                'x': float(pose_data['x']),
                'y': float(pose_data['y']),
                'z': float(pose_data['z'])
            }
        }
        
        # 保存文件
        self.config_file.parent.mkdir(parents=True, exist_ok=True)
        with open(self.config_file, 'w', encoding='utf-8') as f:
            yaml.dump(config, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
        
        self.get_logger().info(f'已保存 {pose_name} - {arm_name}臂 到 {self.config_file}')
        return True


class CalibrationGUI:
    """标定GUI界面"""
    
    def __init__(self, node):
        self.node = node
        
        # 创建主窗口
        self.root = tk.Tk()
        self.root.title("Robot Pose Calibration Tool")
        self.root.geometry("1200x900")
        
        # 配置中文字体
        try:
            import tkinter.font as tkfont
            # Linux常见中文字体
            for font_family in ['WenQuanYi Micro Hei', 'Noto Sans CJK SC', 'Droid Sans Fallback', 
                               'DejaVu Sans', 'Arial Unicode MS', 'Microsoft YaHei']:
                try:
                    test_font = tkfont.Font(family=font_family, size=10)
                    self.default_font = font_family
                    break
                except:
                    continue
            else:
                self.default_font = 'TkDefaultFont'
        except:
            self.default_font = 'TkDefaultFont'
        
        # 关节名称（与URDF匹配）
        self.left_joints = [f'l-j{i}' for i in range(1, 8)]
        self.right_joints = [f'r-j{i}' for i in range(1, 8)]
        
        # 关节角度范围（弧度）
        self.joint_limits = [(-3.14, 3.14)] * 7
        
        # 滑块字典
        self.left_sliders = {}
        self.right_sliders = {}
        
        # 位姿标签
        self.left_pose_labels = {}
        self.right_pose_labels = {}
        
        self._create_widgets()
        
        # 定时更新显示
        self.root.after(100, self._update_display)
    
    def _create_widgets(self):
        """创建界面组件"""
        # 标题
        title = ttk.Label(self.root, text="Robot Pose Calibration Tool", 
                         font=(self.default_font, 16, 'bold'))
        title.pack(pady=10)
        
        # 主容器
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 左右分栏
        left_frame = ttk.LabelFrame(main_frame, text="Left Arm Control", padding=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=5)
        
        right_frame = ttk.LabelFrame(main_frame, text="Right Arm Control", padding=10)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=5)
        
        main_frame.columnconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
        
        # 创建左臂控制
        self._create_arm_control(left_frame, self.left_joints, self.left_sliders, 
                                self.left_pose_labels, 'left')
        
        # 创建右臂控制
        self._create_arm_control(right_frame, self.right_joints, self.right_sliders,
                                self.right_pose_labels, 'right')
        
        # 姿态保存区域
        save_frame = ttk.LabelFrame(self.root, text="Save Pose", padding=10)
        save_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # 姿态选择
        ttk.Label(save_frame, text="Select Pose:").grid(row=0, column=0, padx=5)
        self.pose_var = tk.StringVar(value='arms_open')
        pose_combo = ttk.Combobox(save_frame, textvariable=self.pose_var, 
                                  values=['arms_open', 'arms_down', 'arms_forward', 'arms_forward_low'],
                                  state='readonly', width=20)
        pose_combo.grid(row=0, column=1, padx=5)
        
        # 保存按钮
        ttk.Button(save_frame, text="Save Left Arm", 
                  command=lambda: self._save_current_pose('left')).grid(row=0, column=2, padx=10)
        ttk.Button(save_frame, text="Save Right Arm",
                  command=lambda: self._save_current_pose('right')).grid(row=0, column=3, padx=10)
        ttk.Button(save_frame, text="Save Both Arms",
                  command=self._save_both_poses).grid(row=0, column=4, padx=10)
        
        # 状态栏
        self.status_label = ttk.Label(self.root, text="Ready", relief=tk.SUNKEN, anchor=tk.W)
        self.status_label.pack(side=tk.BOTTOM, fill=tk.X)
    
    def _create_arm_control(self, parent, joint_names, sliders, pose_labels, arm_name):
        """创建单个机械臂的控制界面"""
        # 关节控制区
        joint_frame = ttk.LabelFrame(parent, text="Joint Control", padding=5)
        joint_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        for i, joint_name in enumerate(joint_names):
            # 关节标签
            ttk.Label(joint_frame, text=f"Joint {i+1}:").grid(row=i, column=0, sticky=tk.W, pady=2)
            
            # 滑块
            slider = tk.Scale(joint_frame, from_=self.joint_limits[i][0], 
                            to=self.joint_limits[i][1],
                            resolution=0.01, orient=tk.HORIZONTAL, length=300,
                            command=lambda val, jn=joint_name: self._on_slider_change(jn, val))
            slider.grid(row=i, column=1, padx=5, pady=2)
            slider.set(0.0)
            sliders[joint_name] = slider
            
            # 数值显示
            value_label = ttk.Label(joint_frame, text="0.00", width=8)
            value_label.grid(row=i, column=2, padx=5)
            sliders[f'{joint_name}_label'] = value_label
        
        # 末端位姿显示区
        pose_frame = ttk.LabelFrame(parent, text="End-Effector Pose (base_link)", padding=5)
        pose_frame.pack(fill=tk.X, pady=5)
        
        pose_info = [
            ('X (m):', 'x'),
            ('Y (m):', 'y'),
            ('Z (m):', 'z'),
            ('Roll (°):', 'roll'),
            ('Pitch (°):', 'pitch'),
            ('Yaw (°):', 'yaw')
        ]
        
        for i, (label_text, key) in enumerate(pose_info):
            ttk.Label(pose_frame, text=label_text).grid(row=i, column=0, sticky=tk.W, pady=2)
            value_label = ttk.Label(pose_frame, text="--", width=12, 
                                   font=('Courier', 10))
            value_label.grid(row=i, column=1, sticky=tk.W, padx=5)
            pose_labels[key] = value_label
    
    def _on_slider_change(self, joint_name, value):
        """滑块值改变回调"""
        # 更新数值显示
        label_key = f'{joint_name}_label'
        if joint_name.startswith('left'):
            if label_key in self.left_sliders:
                self.left_sliders[label_key].config(text=f'{float(value):.2f}')
        else:
            if label_key in self.right_sliders:
                self.right_sliders[label_key].config(text=f'{float(value):.2f}')
        
        # 发布关节命令
        self._publish_all_joints()
    
    def _publish_all_joints(self):
        joint_values = {}
        
        # 收集左臂关节
        for joint_name in self.left_joints:
            if joint_name in self.left_sliders:
                joint_values[joint_name] = self.left_sliders[joint_name].get()
        
        # 收集右臂关节
        for joint_name in self.right_joints:
            if joint_name in self.right_sliders:
                joint_values[joint_name] = self.right_sliders[joint_name].get()
        
        # 更新节点中的关节值（由定时器自动发布）
        self.node.update_joint_values
        """定时更新位姿显示"""
        # 更新左臂位姿
        if self.node.left_ee_pose:
            self._update_pose_display(self.node.left_ee_pose, self.left_pose_labels)
        
        # 更新右臂位姿
        if self.node.right_ee_pose:
            self._update_pose_display(self.node.right_ee_pose, self.right_pose_labels)
        
        # 继续定时
        self.root.after(100, self._update_display)
    
    def _update_pose_display(self, pose, labels):
        """更新位姿显示"""
        if pose:
            labels['x'].config(text=f"{pose['x']:.4f}")
            labels['y'].config(text=f"{pose['y']:.4f}")
            labels['z'].config(text=f"{pose['z']:.4f}")
            
            # 四元数转欧拉角
            rot = R.from_quat([pose['qx'], pose['qy'], pose['qz'], pose['qw']])
            euler = rot.as_euler('xyz', degrees=True)
            labels['roll'].config(text=f"{euler[0]:.2f}")
            labels['pitch'].config(text=f"{euler[1]:.2f}")
            labels['yaw'].config(text=f"{euler[2]:.2f}")
    
    def _save_current_pose(self, arm_name):
        """保存当前姿态"""
        pose_name = self.pose_var.get()
        
        # 获取对应臂的位姿
        if arm_name == 'left':
            pose = self.node.left_ee_pose
        else:
            pose = self.node.right_ee_pose
        
        if not pose:
            messagebox.showerror("Error", f"Cannot get {arm_name} arm pose!\nPlease ensure robot is running.")
            return
        
        # 保存
        success = self.node.save_pose(pose_name, arm_name, pose)
        
        if success:
            messagebox.showinfo("Success", 
                f"Saved {pose_name} - {arm_name} arm pose\n"
                f"X: {pose['x']:.4f} m\n"
                f"Y: {pose['y']:.4f} m\n"
                f"Z: {pose['z']:.4f} m")
            self.status_label.config(text=f"Saved {pose_name} - {arm_name} arm")
    
    def _save_both_poses(self):
        """保存双臂位姿"""
        self._save_current_pose('left')
        self._save_current_pose('right')
    
    def run(self):
        """运行GUI"""
        self.root.mainloop()


def main(args=None):
    rclpy.init(args=args)
    
    # 创建节点
    node = RobotPoseCalibrationGUI()
    
    # ROS spin线程
    def spin_thread():
        rclpy.spin(node)
    
    ros_thread = threading.Thread(target=spin_thread, daemon=True)
    ros_thread.start()
    
    # 创建并运行GUI
    gui = CalibrationGUI(node)
    
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
