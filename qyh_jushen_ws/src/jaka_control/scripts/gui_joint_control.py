#!/usr/bin/env python3
"""
JAKA机器人关节控制GUI

提供可视化界面控制机器人关节运动，实时显示关节角度
"""

import sys
import rclpy
from rclpy.node import Node
from jaka_control.srv import MoveJ, EnableRobot, PowerOn, ClearError
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QDoubleSpinBox, QSlider, QGroupBox,
    QGridLayout, QComboBox, QMessageBox, QFrame
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette
import math


class JointControlWidget(QWidget):
    """单个关节控制组件"""
    
    def __init__(self, joint_id, joint_name, parent=None):
        super().__init__(parent)
        self.joint_id = joint_id
        self.joint_name = joint_name
        self.current_angle = 0.0
        
        self.init_ui()
    
    def init_ui(self):
        """初始化UI"""
        layout = QHBoxLayout()
        layout.setSpacing(10)
        
        # 关节名称标签
        name_label = QLabel(f"{self.joint_name}:")
        name_label.setMinimumWidth(80)
        name_label.setFont(QFont("Arial", 10, QFont.Bold))
        layout.addWidget(name_label)
        
        # 减小按钮
        self.btn_decrease = QPushButton("◀")
        self.btn_decrease.setFixedSize(40, 40)
        self.btn_decrease.setStyleSheet("""
            QPushButton {
                background-color: #FF6B6B;
                color: white;
                border-radius: 20px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #FF5252;
            }
            QPushButton:pressed {
                background-color: #E53935;
            }
        """)
        self.btn_decrease.clicked.connect(lambda: self.adjust_angle(-1))
        layout.addWidget(self.btn_decrease)
        
        # 滑动条
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(-180)
        self.slider.setMaximum(180)
        self.slider.setValue(0)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(30)
        self.slider.valueChanged.connect(self.slider_changed)
        layout.addWidget(self.slider, stretch=3)
        
        # 增加按钮
        self.btn_increase = QPushButton("▶")
        self.btn_increase.setFixedSize(40, 40)
        self.btn_increase.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border-radius: 20px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #45A049;
            }
            QPushButton:pressed {
                background-color: #388E3C;
            }
        """)
        self.btn_increase.clicked.connect(lambda: self.adjust_angle(1))
        layout.addWidget(self.btn_increase)
        
        # 当前角度显示
        self.angle_display = QLabel("0.00°")
        self.angle_display.setMinimumWidth(80)
        self.angle_display.setAlignment(Qt.AlignCenter)
        self.angle_display.setFont(QFont("Arial", 12, QFont.Bold))
        self.angle_display.setStyleSheet("""
            QLabel {
                background-color: #2C3E50;
                color: #ECF0F1;
                border-radius: 5px;
                padding: 5px;
            }
        """)
        layout.addWidget(self.angle_display)
        
        # 弧度显示
        self.radian_display = QLabel("0.00 rad")
        self.radian_display.setMinimumWidth(90)
        self.radian_display.setAlignment(Qt.AlignCenter)
        self.radian_display.setFont(QFont("Arial", 9))
        self.radian_display.setStyleSheet("""
            QLabel {
                background-color: #34495E;
                color: #BDC3C7;
                border-radius: 5px;
                padding: 5px;
            }
        """)
        layout.addWidget(self.radian_display)
        
        # 复位按钮
        self.btn_reset = QPushButton("⟲")
        self.btn_reset.setFixedSize(40, 40)
        self.btn_reset.setStyleSheet("""
            QPushButton {
                background-color: #FFA726;
                color: white;
                border-radius: 20px;
                font-size: 18px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #FB8C00;
            }
            QPushButton:pressed {
                background-color: #F57C00;
            }
        """)
        self.btn_reset.clicked.connect(self.reset_angle)
        layout.addWidget(self.btn_reset)
        
        self.setLayout(layout)
    
    def adjust_angle(self, direction):
        """调整角度"""
        step = 5  # 每次调整5度
        new_value = self.slider.value() + (direction * step)
        new_value = max(-180, min(180, new_value))
        self.slider.setValue(new_value)
    
    def slider_changed(self, value):
        """滑动条值改变"""
        self.current_angle = math.radians(value)
        self.update_display()
    
    def reset_angle(self):
        """复位到零"""
        self.slider.setValue(0)
    
    def update_display(self):
        """更新角度显示"""
        degrees = math.degrees(self.current_angle)
        self.angle_display.setText(f"{degrees:.2f}°")
        self.radian_display.setText(f"{self.current_angle:.3f} rad")
    
    def get_angle_rad(self):
        """获取当前角度（弧度）"""
        return self.current_angle
    
    def set_angle_rad(self, angle_rad):
        """设置角度（弧度）"""
        degrees = math.degrees(angle_rad)
        degrees = max(-180, min(180, degrees))
        self.slider.setValue(int(degrees))


class JakaControlGUI(QMainWindow):
    """JAKA机器人控制GUI主窗口"""
    
    def __init__(self):
        super().__init__()
        self.node = None
        self.move_j_client = None
        self.power_client = None
        self.enable_client = None
        self.clear_error_client = None
        
        # 左臂和右臂的关节控制器
        self.left_joints = []
        self.right_joints = []
        
        self.init_ui()
        self.init_ros()
    
    def init_ui(self):
        """初始化用户界面"""
        self.setWindowTitle("JAKA双臂机器人关节控制")
        self.setGeometry(100, 100, 1400, 900)
        
        # 设置主题
        self.setStyleSheet("""
            QMainWindow {
                background-color: #ECF0F1;
            }
            QGroupBox {
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #3498DB;
                border-radius: 10px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
                color: #3498DB;
            }
        """)
        
        # 中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # 标题
        title_label = QLabel("🤖 JAKA双臂机器人关节控制面板")
        title_label.setFont(QFont("Arial", 20, QFont.Bold))
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("""
            QLabel {
                color: #2C3E50;
                padding: 15px;
                background-color: white;
                border-radius: 10px;
            }
        """)
        main_layout.addWidget(title_label)
        
        # 控制按钮区域
        control_layout = self.create_control_panel()
        main_layout.addLayout(control_layout)
        
        # 左臂控制区域
        left_group = self.create_arm_group("左臂 (Left Arm)", "left")
        main_layout.addWidget(left_group)
        
        # 右臂控制区域
        right_group = self.create_arm_group("右臂 (Right Arm)", "right")
        main_layout.addWidget(right_group)
        
        # 底部状态和操作按钮
        bottom_layout = self.create_bottom_panel()
        main_layout.addLayout(bottom_layout)
    
    def create_control_panel(self):
        """创建控制面板"""
        layout = QHBoxLayout()
        
        # 状态显示
        status_group = QGroupBox("系统状态")
        status_layout = QHBoxLayout()
        
        self.status_label = QLabel("未连接")
        self.status_label.setFont(QFont("Arial", 12))
        self.status_label.setStyleSheet("""
            QLabel {
                background-color: #E74C3C;
                color: white;
                padding: 10px 20px;
                border-radius: 5px;
            }
        """)
        status_layout.addWidget(self.status_label)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 机器人选择
        robot_group = QGroupBox("控制目标")
        robot_layout = QHBoxLayout()
        
        robot_layout.addWidget(QLabel("选择机械臂:"))
        self.robot_combo = QComboBox()
        self.robot_combo.addItems(["左臂 (LEFT)", "右臂 (RIGHT)", "双臂 (DUAL)"])
        self.robot_combo.setFont(QFont("Arial", 10))
        self.robot_combo.setStyleSheet("""
            QComboBox {
                padding: 8px;
                border: 2px solid #3498DB;
                border-radius: 5px;
                background-color: white;
            }
        """)
        robot_layout.addWidget(self.robot_combo)
        
        robot_group.setLayout(robot_layout)
        layout.addWidget(robot_group)
        
        # 系统控制按钮
        btn_group = QGroupBox("系统控制")
        btn_layout = QHBoxLayout()
        
        self.btn_clear_error = QPushButton("清除错误")
        self.btn_clear_error.clicked.connect(self.clear_error)
        self.btn_clear_error.setStyleSheet(self.get_button_style("#9B59B6"))
        btn_layout.addWidget(self.btn_clear_error)
        
        self.btn_power = QPushButton("上电")
        self.btn_power.clicked.connect(self.power_on)
        self.btn_power.setStyleSheet(self.get_button_style("#3498DB"))
        btn_layout.addWidget(self.btn_power)
        
        self.btn_enable = QPushButton("使能")
        self.btn_enable.clicked.connect(self.enable_robot)
        self.btn_enable.setStyleSheet(self.get_button_style("#2ECC71"))
        btn_layout.addWidget(self.btn_enable)
        
        btn_group.setLayout(btn_layout)
        layout.addWidget(btn_group)
        
        return layout
    
    def create_arm_group(self, title, arm_type):
        """创建机械臂控制组"""
        group = QGroupBox(title)
        layout = QVBoxLayout()
        layout.setSpacing(5)
        
        joint_names = [
            f"关节 1 (Joint 1)",
            f"关节 2 (Joint 2)",
            f"关节 3 (Joint 3)",
            f"关节 4 (Joint 4)",
            f"关节 5 (Joint 5)",
            f"关节 6 (Joint 6)",
            f"关节 7 (Joint 7)"
        ]
        
        joints_list = self.left_joints if arm_type == "left" else self.right_joints
        
        for i, name in enumerate(joint_names):
            widget = JointControlWidget(i, name)
            layout.addWidget(widget)
            joints_list.append(widget)
        
        group.setLayout(layout)
        return group
    
    def create_bottom_panel(self):
        """创建底部操作面板"""
        layout = QHBoxLayout()
        
        # 快捷操作
        quick_group = QGroupBox("快捷操作")
        quick_layout = QHBoxLayout()
        
        btn_reset_all = QPushButton("全部复位")
        btn_reset_all.clicked.connect(self.reset_all_joints)
        btn_reset_all.setStyleSheet(self.get_button_style("#E67E22"))
        quick_layout.addWidget(btn_reset_all)
        
        btn_home = QPushButton("回到初始位置")
        btn_home.clicked.connect(self.move_to_home)
        btn_home.setStyleSheet(self.get_button_style("#16A085"))
        quick_layout.addWidget(btn_home)
        
        quick_group.setLayout(quick_layout)
        layout.addWidget(quick_group)
        
        # 运动控制
        move_group = QGroupBox("运动控制")
        move_layout = QHBoxLayout()
        
        move_layout.addWidget(QLabel("速度 (rad/s):"))
        self.velocity_spin = QDoubleSpinBox()
        self.velocity_spin.setRange(0.1, 2.0)
        self.velocity_spin.setValue(0.5)
        self.velocity_spin.setSingleStep(0.1)
        self.velocity_spin.setStyleSheet("""
            QDoubleSpinBox {
                padding: 5px;
                border: 2px solid #95A5A6;
                border-radius: 5px;
            }
        """)
        move_layout.addWidget(self.velocity_spin)
        
        move_layout.addWidget(QLabel("加速度 (rad/s²):"))
        self.acceleration_spin = QDoubleSpinBox()
        self.acceleration_spin.setRange(0.1, 3.0)
        self.acceleration_spin.setValue(1.0)
        self.acceleration_spin.setSingleStep(0.1)
        self.acceleration_spin.setStyleSheet("""
            QDoubleSpinBox {
                padding: 5px;
                border: 2px solid #95A5A6;
                border-radius: 5px;
            }
        """)
        move_layout.addWidget(self.acceleration_spin)
        
        self.btn_move = QPushButton("🚀 执行运动")
        self.btn_move.clicked.connect(self.execute_move)
        self.btn_move.setStyleSheet(self.get_button_style("#E74C3C", large=True))
        move_layout.addWidget(self.btn_move)
        
        move_group.setLayout(move_layout)
        layout.addWidget(move_group)
        
        return layout
    
    def get_button_style(self, color, large=False):
        """获取按钮样式"""
        size = "16px" if large else "12px"
        height = "50px" if large else "40px"
        return f"""
            QPushButton {{
                background-color: {color};
                color: white;
                border: none;
                padding: 10px 20px;
                font-size: {size};
                font-weight: bold;
                border-radius: 5px;
                min-height: {height};
            }}
            QPushButton:hover {{
                background-color: {self.adjust_color(color, -20)};
            }}
            QPushButton:pressed {{
                background-color: {self.adjust_color(color, -40)};
            }}
        """
    
    def adjust_color(self, hex_color, amount):
        """调整颜色亮度"""
        # 简单的颜色调整，实际使用可以更复杂
        return hex_color
    
    def init_ros(self):
        """初始化ROS"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self.node = Node('jaka_gui_control')
            
            # 创建服务客户端
            self.move_j_client = self.node.create_client(
                MoveJ, '/jaka_robot_node/move_j')
            self.power_client = self.node.create_client(
                PowerOn, '/jaka_robot_node/power_on')
            self.enable_client = self.node.create_client(
                EnableRobot, '/jaka_robot_node/enable_robot')
            self.clear_error_client = self.node.create_client(
                ClearError, '/jaka_robot_node/clear_error')
            
            # 启动定时器检查服务状态
            self.timer = QTimer()
            self.timer.timeout.connect(self.check_service_status)
            self.timer.start(1000)  # 每秒检查一次
            
        except Exception as e:
            QMessageBox.critical(self, "错误", f"初始化ROS失败: {str(e)}")
    
    def check_service_status(self):
        """检查服务状态"""
        if self.move_j_client and self.move_j_client.service_is_ready():
            self.status_label.setText("✓ 已连接")
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #2ECC71;
                    color: white;
                    padding: 10px 20px;
                    border-radius: 5px;
                }
            """)
        else:
            self.status_label.setText("✗ 服务未就绪")
            self.status_label.setStyleSheet("""
                QLabel {
                    background-color: #E74C3C;
                    color: white;
                    padding: 10px 20px;
                    border-radius: 5px;
                }
            """)
    
    def get_joint_positions(self):
        """获取所有关节位置（14个值）"""
        positions = []
        
        # 左臂7个关节
        for joint in self.left_joints:
            positions.append(joint.get_angle_rad())
        
        # 右臂7个关节
        for joint in self.right_joints:
            positions.append(joint.get_angle_rad())
        
        return positions
    
    def get_robot_id(self):
        """获取选择的机器人ID"""
        index = self.robot_combo.currentIndex()
        return [0, 1, -1][index]  # LEFT=0, RIGHT=1, DUAL=-1
    
    def clear_error(self):
        """清除错误"""
        if not self.clear_error_client or not self.clear_error_client.service_is_ready():
            QMessageBox.warning(self, "警告", "清除错误服务未就绪")
            return
        
        request = ClearError.Request()
        future = self.clear_error_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.result():
            QMessageBox.information(self, "成功", "错误已清除")
        else:
            QMessageBox.warning(self, "失败", "清除错误失败")
    
    def power_on(self):
        """上电"""
        if not self.power_client or not self.power_client.service_is_ready():
            QMessageBox.warning(self, "警告", "上电服务未就绪")
            return
        
        request = PowerOn.Request()
        future = self.power_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            QMessageBox.information(self, "成功", "机器人已上电")
        else:
            msg = future.result().message if future.result() else "超时"
            QMessageBox.warning(self, "失败", f"上电失败: {msg}")
    
    def enable_robot(self):
        """使能"""
        if not self.enable_client or not self.enable_client.service_is_ready():
            QMessageBox.warning(self, "警告", "使能服务未就绪")
            return
        
        request = EnableRobot.Request()
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            QMessageBox.information(self, "成功", "机器人已使能")
        else:
            msg = future.result().message if future.result() else "超时"
            QMessageBox.warning(self, "失败", f"使能失败: {msg}")
    
    def reset_all_joints(self):
        """重置所有关节到零位"""
        for joint in self.left_joints + self.right_joints:
            joint.reset_angle()
    
    def move_to_home(self):
        """移动到初始位置"""
        self.reset_all_joints()
        self.execute_move()
    
    def execute_move(self):
        """执行运动"""
        if not self.move_j_client or not self.move_j_client.service_is_ready():
            QMessageBox.warning(self, "警告", "运动服务未就绪，请确保机器人节点正在运行")
            return
        
        # 获取14个关节位置
        joint_positions = self.get_joint_positions()
        robot_id = self.get_robot_id()
        
        # 创建请求
        request = MoveJ.Request()
        request.robot_id = robot_id
        request.joint_positions = joint_positions
        request.move_mode = False  # 绝对运动
        request.velocity = self.velocity_spin.value()
        request.acceleration = self.acceleration_spin.value()
        request.is_block = False  # 非阻塞
        
        # 发送请求
        future = self.move_j_client.call_async(request)
        
        # 显示发送信息
        robot_names = ["左臂", "右臂", "双臂"]
        robot_name = robot_names[self.robot_combo.currentIndex()]
        
        QMessageBox.information(
            self, 
            "运动指令已发送", 
            f"目标: {robot_name}\n"
            f"速度: {request.velocity:.2f} rad/s\n"
            f"加速度: {request.acceleration:.2f} rad/s²\n\n"
            f"关节角度:\n" + 
            "\n".join([f"  左臂关节{i+1}: {joint_positions[i]:.3f} rad ({math.degrees(joint_positions[i]):.1f}°)" 
                      for i in range(7)]) + "\n" +
            "\n".join([f"  右臂关节{i+1}: {joint_positions[i+7]:.3f} rad ({math.degrees(joint_positions[i+7]):.1f}°)" 
                      for i in range(7)])
        )
    
    def closeEvent(self, event):
        """关闭窗口时清理"""
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()


def main():
    """主函数"""
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle('Fusion')
    
    window = JakaControlGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
