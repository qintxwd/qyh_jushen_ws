#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rclpy
from rclpy.node import Node
from qyh_gripper_msgs.msg import GripperState
from qyh_gripper_msgs.srv import ActivateGripper, MoveGripper, GetGripperState
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QLabel, QGroupBox, QGridLayout,
                             QPushButton, QSlider, QSpinBox, QProgressBar)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor


class GripperSignals(QObject):
    """信号类，用于线程安全的UI更新"""
    left_state_updated = pyqtSignal(object)
    right_state_updated = pyqtSignal(object)


class GripperMonitorNode(Node):
    def __init__(self, signals):
        super().__init__('gripper_monitor_node')
        self.signals = signals
        
        # 左手夹爪订阅
        self.left_sub = self.create_subscription(
            GripperState,
            '/left/gripper_state',
            lambda msg: self.signals.left_state_updated.emit(msg),
            10)
        
        # 右手夹爪订阅
        self.right_sub = self.create_subscription(
            GripperState,
            '/right/gripper_state',
            lambda msg: self.signals.right_state_updated.emit(msg),
            10)
        
        # 左手服务客户端
        self.left_activate_client = self.create_client(
            ActivateGripper, '/left/activate_gripper')
        self.left_move_client = self.create_client(
            MoveGripper, '/left/move_gripper')
        
        # 右手服务客户端
        self.right_activate_client = self.create_client(
            ActivateGripper, '/right/activate_gripper')
        self.right_move_client = self.create_client(
            MoveGripper, '/right/move_gripper')
        
        self.get_logger().info('Gripper Monitor Node started')
    
    def call_activate(self, side):
        """激活夹爪"""
        client = self.left_activate_client if side == 'left' else self.right_activate_client
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=1.0)
        
        request = ActivateGripper.Request()
        future = client.call_async(request)
        return future
    
    def call_move(self, side, position, speed, force):
        """移动夹爪"""
        client = self.left_move_client if side == 'left' else self.right_move_client
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=1.0)
        
        request = MoveGripper.Request()
        request.position = int(position)
        request.speed = int(speed)
        request.force = int(force)
        
        future = client.call_async(request)
        return future


class GripperControlPanel(QWidget):
    """单个夹爪的控制面板"""
    
    def __init__(self, side, ros_node, parent=None):
        super().__init__(parent)
        self.side = side  # 'left' or 'right'
        self.ros_node = ros_node
        self.current_state = None
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(10)
        
        # 标题
        title_label = QLabel(f'{self.side.upper()} Gripper')
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(title_label)
        
        # 状态显示组
        status_group = self.create_status_group()
        layout.addWidget(status_group)
        
        # 控制组
        control_group = self.create_control_group()
        layout.addWidget(control_group)
        
        # 快捷按钮组
        quick_actions_group = self.create_quick_actions_group()
        layout.addWidget(quick_actions_group)
        
        layout.addStretch()
        self.setLayout(layout)
    
    def create_status_group(self):
        """创建状态显示组"""
        group = QGroupBox('状态监控')
        grid = QGridLayout()
        
        # 激活状态
        grid.addWidget(QLabel('激活状态:'), 0, 0)
        self.status_activated = QLabel('未激活')
        self.status_activated.setStyleSheet('color: red; font-weight: bold;')
        grid.addWidget(self.status_activated, 0, 1)
        
        # 通信状态
        grid.addWidget(QLabel('通信状态:'), 1, 0)
        self.status_comm = QLabel('断开')
        self.status_comm.setStyleSheet('color: red; font-weight: bold;')
        grid.addWidget(self.status_comm, 1, 1)
        
        # 物体检测
        grid.addWidget(QLabel('物体检测:'), 2, 0)
        self.status_object = QLabel('-')
        grid.addWidget(self.status_object, 2, 1)
        
        # 当前位置
        grid.addWidget(QLabel('当前位置:'), 3, 0)
        self.status_position = QLabel('0')
        grid.addWidget(self.status_position, 3, 1)
        
        # 位置进度条
        self.position_bar = QProgressBar()
        self.position_bar.setMaximum(255)
        self.position_bar.setFormat('%v / 255')
        grid.addWidget(self.position_bar, 4, 0, 1, 2)
        
        # 当前力反馈
        grid.addWidget(QLabel('力反馈:'), 5, 0)
        self.status_force = QLabel('0')
        grid.addWidget(self.status_force, 5, 1)
        
        # 力反馈进度条
        self.force_bar = QProgressBar()
        self.force_bar.setMaximum(255)
        self.force_bar.setFormat('%v / 255')
        self.force_bar.setStyleSheet("""
            QProgressBar::chunk {
                background-color: #FFA500;
            }
        """)
        grid.addWidget(self.force_bar, 6, 0, 1, 2)
        
        # 故障信息
        grid.addWidget(QLabel('故障信息:'), 7, 0)
        self.status_fault = QLabel('无')
        grid.addWidget(self.status_fault, 7, 1)
        
        group.setLayout(grid)
        return group
    
    def create_control_group(self):
        """创建控制组"""
        group = QGroupBox('精确控制')
        layout = QVBoxLayout()
        
        # 激活按钮
        self.btn_activate = QPushButton('激活夹爪')
        self.btn_activate.clicked.connect(self.on_activate)
        self.btn_activate.setStyleSheet('background-color: #4CAF50; padding: 10px;')
        layout.addWidget(self.btn_activate)
        
        # 位置控制
        pos_layout = QHBoxLayout()
        pos_layout.addWidget(QLabel('目标位置 (0=开, 255=闭):'))
        self.pos_slider = QSlider(Qt.Horizontal)
        self.pos_slider.setMinimum(0)
        self.pos_slider.setMaximum(255)
        self.pos_slider.setValue(0)
        self.pos_slider.valueChanged.connect(lambda v: self.pos_spinbox.setValue(v))
        pos_layout.addWidget(self.pos_slider)
        self.pos_spinbox = QSpinBox()
        self.pos_spinbox.setMinimum(0)
        self.pos_spinbox.setMaximum(255)
        self.pos_spinbox.setValue(0)
        self.pos_spinbox.valueChanged.connect(lambda v: self.pos_slider.setValue(v))
        pos_layout.addWidget(self.pos_spinbox)
        layout.addLayout(pos_layout)
        
        # 速度控制
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel('速度 (0-255):'))
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setMinimum(0)
        self.speed_slider.setMaximum(255)
        self.speed_slider.setValue(255)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_spinbox.setValue(v))
        speed_layout.addWidget(self.speed_slider)
        self.speed_spinbox = QSpinBox()
        self.speed_spinbox.setMinimum(0)
        self.speed_spinbox.setMaximum(255)
        self.speed_spinbox.setValue(255)
        self.speed_spinbox.valueChanged.connect(lambda v: self.speed_slider.setValue(v))
        speed_layout.addWidget(self.speed_spinbox)
        layout.addLayout(speed_layout)
        
        # 力控制
        force_layout = QHBoxLayout()
        force_layout.addWidget(QLabel('力限制 (0-255):'))
        self.force_slider = QSlider(Qt.Horizontal)
        self.force_slider.setMinimum(0)
        self.force_slider.setMaximum(255)
        self.force_slider.setValue(150)
        self.force_slider.valueChanged.connect(lambda v: self.force_spinbox.setValue(v))
        force_layout.addWidget(self.force_slider)
        self.force_spinbox = QSpinBox()
        self.force_spinbox.setMinimum(0)
        self.force_spinbox.setMaximum(255)
        self.force_spinbox.setValue(150)
        self.force_spinbox.valueChanged.connect(lambda v: self.force_slider.setValue(v))
        force_layout.addWidget(self.force_spinbox)
        layout.addLayout(force_layout)
        
        # 执行按钮
        self.btn_move = QPushButton('执行移动')
        self.btn_move.clicked.connect(self.on_move)
        self.btn_move.setStyleSheet('background-color: #2196F3; padding: 10px;')
        layout.addWidget(self.btn_move)
        
        group.setLayout(layout)
        return group
    
    def create_quick_actions_group(self):
        """创建快捷操作组"""
        group = QGroupBox('快捷操作')
        layout = QGridLayout()
        
        # 全开
        btn_open = QPushButton('全开')
        btn_open.clicked.connect(lambda: self.quick_move(0, 255, 255))
        btn_open.setStyleSheet('background-color: #00BCD4; padding: 8px;')
        layout.addWidget(btn_open, 0, 0)
        
        # 全闭
        btn_close = QPushButton('全闭')
        btn_close.clicked.connect(lambda: self.quick_move(255, 255, 150))
        btn_close.setStyleSheet('background-color: #FF5722; padding: 8px;')
        layout.addWidget(btn_close, 0, 1)
        
        # 半开
        btn_half = QPushButton('半开')
        btn_half.clicked.connect(lambda: self.quick_move(128, 255, 150))
        btn_half.setStyleSheet('background-color: #FFC107; padding: 8px;')
        layout.addWidget(btn_half, 1, 0)
        
        # 轻抓
        btn_soft = QPushButton('轻抓(柔性)')
        btn_soft.clicked.connect(lambda: self.quick_move(255, 100, 50))
        btn_soft.setStyleSheet('background-color: #9C27B0; padding: 8px;')
        layout.addWidget(btn_soft, 1, 1)
        
        group.setLayout(layout)
        return group
    
    def on_activate(self):
        """激活夹爪"""
        self.btn_activate.setEnabled(False)
        self.btn_activate.setText('激活中...')
        try:
            future = self.ros_node.call_activate(self.side)
            # 简单处理，实际应该等待结果
        except Exception as e:
            print(f"Activate error: {e}")
        finally:
            QTimer.singleShot(2000, lambda: self.btn_activate.setEnabled(True))
            QTimer.singleShot(2000, lambda: self.btn_activate.setText('激活夹爪'))
    
    def on_move(self):
        """执行移动"""
        position = self.pos_spinbox.value()
        speed = self.speed_spinbox.value()
        force = self.force_spinbox.value()
        self.quick_move(position, speed, force)
    
    def quick_move(self, position, speed, force):
        """快捷移动"""
        try:
            future = self.ros_node.call_move(self.side, position, speed, force)
        except Exception as e:
            print(f"Move error: {e}")
    
    def update_state(self, state: GripperState):
        """更新状态显示"""
        self.current_state = state
        
        # 激活状态
        if state.is_activated:
            self.status_activated.setText('已激活')
            self.status_activated.setStyleSheet('color: green; font-weight: bold;')
            self.btn_move.setEnabled(True)
        else:
            self.status_activated.setText('未激活')
            self.status_activated.setStyleSheet('color: red; font-weight: bold;')
            self.btn_move.setEnabled(False)
        
        # 通信状态
        if state.communication_ok:
            self.status_comm.setText('正常')
            self.status_comm.setStyleSheet('color: green; font-weight: bold;')
        else:
            self.status_comm.setText('断开')
            self.status_comm.setStyleSheet('color: red; font-weight: bold;')
        
        # 物体检测
        object_status_map = {
            0: '运动中',
            1: '内撑检测到',
            2: '外夹抓到',
            3: '到达位置'
        }
        obj_text = object_status_map.get(state.object_status, '未知')
        self.status_object.setText(obj_text)
        if state.object_status == 2:
            self.status_object.setStyleSheet('color: green; font-weight: bold;')
        else:
            self.status_object.setStyleSheet('color: black;')
        
        # 位置
        self.status_position.setText(f'{state.current_position}')
        self.position_bar.setValue(state.current_position)
        
        # 力反馈
        self.status_force.setText(f'{state.current_force}')
        self.force_bar.setValue(state.current_force)
        
        # 故障
        if state.fault_code == 0:
            self.status_fault.setText('无')
            self.status_fault.setStyleSheet('color: green;')
        else:
            self.status_fault.setText(state.fault_message)
            self.status_fault.setStyleSheet('color: red; font-weight: bold;')


class GripperGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('JODELL EPG 双手夹爪控制')
        self.setGeometry(100, 100, 1200, 800)
        
        # 设置样式
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f5f5f5;
            }
            QGroupBox {
                font-weight: bold;
                border: 2px solid #4a90e2;
                border-radius: 5px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                color: white;
                border: none;
                border-radius: 5px;
                font-weight: bold;
            }
            QPushButton:hover {
                opacity: 0.8;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
        """)
        
        # 初始化ROS2
        self.signals = GripperSignals()
        
        rclpy.init()
        self.ros_node = GripperMonitorNode(self.signals)
        
        # 先创建UI（这样left_panel和right_panel才存在）
        self.init_ui()
        
        # 然后连接信号
        self.signals.left_state_updated.connect(self.left_panel.update_state)
        self.signals.right_state_updated.connect(self.right_panel.update_state)
        
        # 创建定时器来处理ROS2回调
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros_node)
        self.ros_timer.start(50)  # 20Hz
    
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout()
        
        # 标题
        title = QLabel('JODELL EPG 双手夹爪控制系统')
        title_font = QFont()
        title_font.setPointSize(18)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet('color: #2c5aa0; padding: 15px;')
        main_layout.addWidget(title)
        
        # 双手夹爪控制面板
        grippers_layout = QHBoxLayout()
        
        # 左手
        self.left_panel = GripperControlPanel('left', self.ros_node)
        grippers_layout.addWidget(self.left_panel)
        
        # 右手
        self.right_panel = GripperControlPanel('right', self.ros_node)
        grippers_layout.addWidget(self.right_panel)
        
        main_layout.addLayout(grippers_layout)
        
        central_widget.setLayout(main_layout)
    
    def spin_ros_node(self):
        if rclpy.ok():
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0)
            except Exception:
                pass
    
    def closeEvent(self, event):
        """关闭窗口时清理资源"""
        self.ros_timer.stop()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main(args=None):
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    
    window = GripperGUI()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
