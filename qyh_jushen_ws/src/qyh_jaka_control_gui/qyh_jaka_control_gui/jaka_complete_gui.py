#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
JAKA双臂机器人完整控制GUI
集成所有基础控制和VR跟随功能
"""
import sys
import signal
from datetime import datetime
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QGroupBox, QGridLayout, QPushButton,
    QLineEdit, QTextEdit, QTabWidget, QDoubleSpinBox
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from qyh_jaka_control_msgs.msg import (
    JakaDualJointServo,
    JakaDualCartesianServo,
    JakaServoStatus,
    VRFollowStatus,
    RobotState
)
from qyh_jaka_control_msgs.srv import (
    StartServo, StopServo,
    EnableVRFollow, CalibrateVR,
    MoveJ, MoveL,
    SetCollisionLevel, SetToolOffset,
    GetRobotState
)


class SignalBridge(QObject):
    """线程安全的信号桥接"""
    status_updated = pyqtSignal(object)
    vr_status_updated = pyqtSignal(object)
    robot_state_updated = pyqtSignal(object)
    joint_state_updated = pyqtSignal(object)
    log_message = pyqtSignal(str, str)


class JakaControlNode(Node):
    def __init__(self, signals):
        super().__init__('jaka_control_gui')
        self.signals = signals

        # Publishers
        self.joint_pub = self.create_publisher(
            JakaDualJointServo, '/jaka/servo/joint_cmd', 1)
        self.cart_pub = self.create_publisher(
            JakaDualCartesianServo, '/jaka/servo/cartesian_cmd', 1)

        # Subscribers
        self.status_sub = self.create_subscription(
            JakaServoStatus, '/jaka/servo/status',
            self.status_callback, 10)
        self.vr_status_sub = self.create_subscription(
            VRFollowStatus, '/jaka/vr/status',
            self.vr_status_callback, 10)
        self.robot_state_sub = self.create_subscription(
            RobotState, '/jaka/robot_state',
            self.robot_state_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states',
            self.joint_state_callback, 10)

        # 基础控制服务 (假设通过ROS服务调用底层接口)
        self.cli_power_on = self.create_client(
            Trigger, '/jaka/robot/power_on')
        self.cli_power_off = self.create_client(
            Trigger, '/jaka/robot/power_off')
        self.cli_enable = self.create_client(
            Trigger, '/jaka/robot/enable')
        self.cli_disable = self.create_client(
            Trigger, '/jaka/robot/disable')
        self.cli_clear_error = self.create_client(
            Trigger, '/jaka/robot/clear_error')
        self.cli_motion_abort = self.create_client(
            Trigger, '/jaka/robot/motion_abort')

        # 伺服控制服务
        self.cli_start_servo = self.create_client(
            StartServo, '/jaka/servo/start')
        self.cli_stop_servo = self.create_client(
            StopServo, '/jaka/servo/stop')

        # VR控制服务
        self.cli_enable_vr = self.create_client(
            EnableVRFollow, '/jaka/vr/enable')
        self.cli_calibrate_vr = self.create_client(
            CalibrateVR, '/jaka/vr/calibrate')

        # 点到点运动服务
        self.cli_move_j = self.create_client(
            MoveJ, '/jaka/move_j')
        self.cli_move_l = self.create_client(
            MoveL, '/jaka/move_l')

        # 配置服务
        self.cli_set_collision = self.create_client(
            SetCollisionLevel, '/jaka/set_collision_level')
        self.cli_set_tool = self.create_client(
            SetToolOffset, '/jaka/set_tool_offset')
        self.cli_get_state = self.create_client(
            GetRobotState, '/jaka/get_robot_state')

    def status_callback(self, msg):
        self.signals.status_updated.emit(msg)

    def vr_status_callback(self, msg):
        self.signals.vr_status_updated.emit(msg)

    def robot_state_callback(self, msg):
        self.signals.robot_state_updated.emit(msg)

    def joint_state_callback(self, msg):
        self.signals.joint_state_updated.emit(msg)

    def call_service_async(self, client, request):
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=1.0)
        return client.call_async(request)


class JakaControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('JAKA双臂机器人控制系统')
        self.setGeometry(100, 100, 1200, 900)

        # 初始化ROS2
        self.signals = SignalBridge()
        self.signals.status_updated.connect(self.update_status)
        self.signals.vr_status_updated.connect(self.update_vr_status)
        self.signals.robot_state_updated.connect(self.update_robot_state)
        self.signals.joint_state_updated.connect(self.update_joint_state)
        self.signals.log_message.connect(self.add_log)

        rclpy.init()
        self.ros_node = JakaControlNode(self.signals)

        # ROS spin timer
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        self.ros_timer.start(10)  # 100Hz

        # 初始化UI
        self.init_ui()

        # 状态变量
        self.powered_on = False
        self.enabled = False
        self.servo_running = False
        self.vr_following = False

        self.add_log("JAKA双臂机器人控制系统已启动", "info")

    def init_ui(self):
        self.setStyleSheet("""
            QMainWindow {
                background: qlineargradient(x1:0, y1:0, x2:0, y2:1,
                    stop:0 #e8f4f8, stop:1 #d5e8f0);
            }
            QGroupBox {
                font-weight: bold;
                font-size: 11pt;
                border: 2px solid #4a90e2;
                border-radius: 8px;
                margin-top: 16px;
                padding-top: 12px;
                background-color: white;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top left;
                left: 15px;
                padding: 0 8px;
                color: #2c5aa0;
            }
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-weight: bold;
                font-size: 10pt;
                min-width: 120px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3d8b40;
            }
            QPushButton:disabled {
                background-color: #cccccc;
                color: #666666;
            }
            QPushButton#stopButton {
                background-color: #f44336;
            }
            QPushButton#stopButton:hover {
                background-color: #da190b;
            }
            QPushButton#warningButton {
                background-color: #FF9800;
            }
            QPushButton#warningButton:hover {
                background-color: #e68900;
            }
            QPushButton#infoButton {
                background-color: #2196F3;
            }
            QPushButton#infoButton:hover {
                background-color: #0b7dda;
            }
        """)

        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(15, 15, 15, 15)

        # 标题
        header = QLabel('◆ JAKA双臂机器人控制系统 ◆')
        header_font = QFont()
        header_font.setPointSize(20)
        header_font.setBold(True)
        header.setFont(header_font)
        header.setStyleSheet('color: #2c5aa0; background: transparent;')
        header.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(header)

        # 标签页
        tabs = QTabWidget()
        tabs.addTab(self.create_basic_control_tab(), "基础控制")
        tabs.addTab(self.create_motion_tab(), "点到点运动")
        tabs.addTab(self.create_servo_tab(), "伺服模式")
        tabs.addTab(self.create_vr_tab(), "VR跟随")
        tabs.addTab(self.create_config_tab(), "参数配置")
        tabs.addTab(self.create_status_tab(), "状态监控")
        main_layout.addWidget(tabs)

        # 日志区域
        log_group = QGroupBox('● 系统日志')
        log_layout = QVBoxLayout()
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        log_layout.addWidget(self.log_text)
        log_group.setLayout(log_layout)
        main_layout.addWidget(log_group)

    def create_basic_control_tab(self):
        """基础控制标签页：电源、使能、清除错误等"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # 连接状态组
        conn_group = QGroupBox('● 连接状态')
        conn_layout = QGridLayout()

        self.left_ip_label = QLabel('192.168.1.10')
        self.right_ip_label = QLabel('192.168.1.11')
        self.left_conn_status = QLabel('未连接')
        self.right_conn_status = QLabel('未连接')

        conn_layout.addWidget(QLabel('左臂IP:'), 0, 0)
        conn_layout.addWidget(self.left_ip_label, 0, 1)
        conn_layout.addWidget(self.left_conn_status, 0, 2)
        conn_layout.addWidget(QLabel('右臂IP:'), 1, 0)
        conn_layout.addWidget(self.right_ip_label, 1, 1)
        conn_layout.addWidget(self.right_conn_status, 1, 2)

        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)

        # 电源和使能组
        power_group = QGroupBox('● 电源与使能')
        power_layout = QGridLayout()

        self.btn_power_on = QPushButton('上电 (Power On)')
        self.btn_power_on.clicked.connect(self.power_on)

        self.btn_power_off = QPushButton('下电 (Power Off)')
        self.btn_power_off.setObjectName("stopButton")
        self.btn_power_off.clicked.connect(self.power_off)

        self.btn_enable = QPushButton('使能 (Enable)')
        self.btn_enable.clicked.connect(self.enable_robot)
        self.btn_enable.setEnabled(False)

        self.btn_disable = QPushButton('去使能 (Disable)')
        self.btn_disable.setObjectName("warningButton")
        self.btn_disable.clicked.connect(self.disable_robot)
        self.btn_disable.setEnabled(False)

        self.power_status_label = QLabel('未上电')
        self.enable_status_label = QLabel('未使能')

        power_layout.addWidget(QLabel('电源状态:'), 0, 0)
        power_layout.addWidget(self.power_status_label, 0, 1, 1, 2)
        power_layout.addWidget(self.btn_power_on, 1, 0)
        power_layout.addWidget(self.btn_power_off, 1, 1)

        power_layout.addWidget(QLabel('使能状态:'), 2, 0)
        power_layout.addWidget(self.enable_status_label, 2, 1, 1, 2)
        power_layout.addWidget(self.btn_enable, 3, 0)
        power_layout.addWidget(self.btn_disable, 3, 1)

        power_group.setLayout(power_layout)
        layout.addWidget(power_group)

        # 错误和急停组
        error_group = QGroupBox('● 错误处理')
        error_layout = QGridLayout()

        self.btn_clear_error = QPushButton('清除错误')
        self.btn_clear_error.setObjectName("warningButton")
        self.btn_clear_error.clicked.connect(self.clear_error)

        self.btn_motion_abort = QPushButton('急停 (Abort)')
        self.btn_motion_abort.setObjectName("stopButton")
        self.btn_motion_abort.clicked.connect(self.motion_abort)

        self.error_status_label = QLabel('无错误')

        error_layout.addWidget(QLabel('错误状态:'), 0, 0)
        error_layout.addWidget(self.error_status_label, 0, 1, 1, 2)
        error_layout.addWidget(self.btn_clear_error, 1, 0)
        error_layout.addWidget(self.btn_motion_abort, 1, 1)

        error_group.setLayout(error_layout)
        layout.addWidget(error_group)

        layout.addStretch()
        return widget

    def create_motion_tab(self):
        """点到点运动标签页"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # MoveJ 关节运动组
        movej_group = QGroupBox('● 关节空间运动 (MoveJ)')
        movej_layout = QGridLayout()

        self.movej_robot_id = QDoubleSpinBox()
        self.movej_robot_id.setRange(-1, 1)
        self.movej_robot_id.setValue(0)
        self.movej_robot_id.setDecimals(0)
        self.movej_robot_id.setSuffix('')

        self.movej_positions = QLineEdit('0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.movej_velocity = QDoubleSpinBox()
        self.movej_velocity.setRange(0.01, 3.14)
        self.movej_velocity.setValue(1.0)
        self.movej_velocity.setSuffix(' rad/s')
        
        self.movej_accel = QDoubleSpinBox()
        self.movej_accel.setRange(0.01, 10.0)
        self.movej_accel.setValue(0.5)
        self.movej_accel.setSuffix(' rad/s²')

        self.btn_move_j = QPushButton('执行 MoveJ')
        self.btn_move_j.setObjectName("infoButton")
        self.btn_move_j.clicked.connect(self.execute_move_j)
        self.btn_move_j.setEnabled(False)

        movej_layout.addWidget(QLabel('机器人ID (-1:双臂, 0:左, 1:右):'), 0, 0)
        movej_layout.addWidget(self.movej_robot_id, 0, 1)
        movej_layout.addWidget(QLabel('关节位置 (14个值，逗号分隔):'), 1, 0)
        movej_layout.addWidget(self.movej_positions, 1, 1)
        movej_layout.addWidget(QLabel('速度:'), 2, 0)
        movej_layout.addWidget(self.movej_velocity, 2, 1)
        movej_layout.addWidget(QLabel('加速度:'), 3, 0)
        movej_layout.addWidget(self.movej_accel, 3, 1)
        movej_layout.addWidget(self.btn_move_j, 4, 0, 1, 2)

        movej_group.setLayout(movej_layout)
        layout.addWidget(movej_group)

        # MoveL 直线运动组
        movel_group = QGroupBox('● 笛卡尔空间直线运动 (MoveL)')
        movel_layout = QGridLayout()

        self.movel_robot_id = QDoubleSpinBox()
        self.movel_robot_id.setRange(0, 1)
        self.movel_robot_id.setValue(0)
        self.movel_robot_id.setDecimals(0)

        self.movel_x = QDoubleSpinBox()
        self.movel_x.setRange(-2.0, 2.0)
        self.movel_x.setValue(0.5)
        self.movel_x.setSuffix(' m')
        self.movel_x.setDecimals(3)

        self.movel_y = QDoubleSpinBox()
        self.movel_y.setRange(-2.0, 2.0)
        self.movel_y.setValue(0.0)
        self.movel_y.setSuffix(' m')
        self.movel_y.setDecimals(3)

        self.movel_z = QDoubleSpinBox()
        self.movel_z.setRange(-2.0, 2.0)
        self.movel_z.setValue(0.3)
        self.movel_z.setSuffix(' m')
        self.movel_z.setDecimals(3)

        self.movel_velocity = QDoubleSpinBox()
        self.movel_velocity.setRange(1.0, 500.0)
        self.movel_velocity.setValue(100.0)
        self.movel_velocity.setSuffix(' mm/s')

        self.movel_accel = QDoubleSpinBox()
        self.movel_accel.setRange(1.0, 1000.0)
        self.movel_accel.setValue(50.0)
        self.movel_accel.setSuffix(' mm/s²')

        self.btn_move_l = QPushButton('执行 MoveL')
        self.btn_move_l.setObjectName("infoButton")
        self.btn_move_l.clicked.connect(self.execute_move_l)
        self.btn_move_l.setEnabled(False)

        movel_layout.addWidget(QLabel('机器人ID (0:左, 1:右):'), 0, 0)
        movel_layout.addWidget(self.movel_robot_id, 0, 1)
        movel_layout.addWidget(QLabel('目标位置 X:'), 1, 0)
        movel_layout.addWidget(self.movel_x, 1, 1)
        movel_layout.addWidget(QLabel('目标位置 Y:'), 2, 0)
        movel_layout.addWidget(self.movel_y, 2, 1)
        movel_layout.addWidget(QLabel('目标位置 Z:'), 3, 0)
        movel_layout.addWidget(self.movel_z, 3, 1)
        movel_layout.addWidget(QLabel('速度:'), 4, 0)
        movel_layout.addWidget(self.movel_velocity, 4, 1)
        movel_layout.addWidget(QLabel('加速度:'), 5, 0)
        movel_layout.addWidget(self.movel_accel, 5, 1)
        movel_layout.addWidget(self.btn_move_l, 6, 0, 1, 2)

        movel_group.setLayout(movel_layout)
        layout.addWidget(movel_group)

        layout.addStretch()
        return widget

    def create_config_tab(self):
        """参数配置标签页"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # 碰撞检测配置组
        collision_group = QGroupBox('● 碰撞检测等级')
        collision_layout = QGridLayout()

        self.collision_robot_id = QDoubleSpinBox()
        self.collision_robot_id.setRange(0, 1)
        self.collision_robot_id.setValue(0)
        self.collision_robot_id.setDecimals(0)

        self.collision_level = QDoubleSpinBox()
        self.collision_level.setRange(0, 5)
        self.collision_level.setValue(3)
        self.collision_level.setDecimals(0)

        self.btn_set_collision = QPushButton('设置碰撞等级')
        self.btn_set_collision.setObjectName("warningButton")
        self.btn_set_collision.clicked.connect(self.set_collision_level)

        collision_layout.addWidget(QLabel('机器人ID (0:左, 1:右):'), 0, 0)
        collision_layout.addWidget(self.collision_robot_id, 0, 1)
        collision_layout.addWidget(QLabel('碰撞等级 (0-5, 0最灵敏):'), 1, 0)
        collision_layout.addWidget(self.collision_level, 1, 1)
        collision_layout.addWidget(self.btn_set_collision, 2, 0, 1, 2)

        collision_group.setLayout(collision_layout)
        layout.addWidget(collision_group)

        # 工具偏移配置组
        tool_group = QGroupBox('● 工具坐标系偏移')
        tool_layout = QGridLayout()

        self.tool_robot_id = QDoubleSpinBox()
        self.tool_robot_id.setRange(0, 1)
        self.tool_robot_id.setValue(0)
        self.tool_robot_id.setDecimals(0)

        self.tool_offset_x = QDoubleSpinBox()
        self.tool_offset_x.setRange(-0.5, 0.5)
        self.tool_offset_x.setValue(0.0)
        self.tool_offset_x.setSuffix(' m')
        self.tool_offset_x.setDecimals(3)

        self.tool_offset_y = QDoubleSpinBox()
        self.tool_offset_y.setRange(-0.5, 0.5)
        self.tool_offset_y.setValue(0.0)
        self.tool_offset_y.setSuffix(' m')
        self.tool_offset_y.setDecimals(3)

        self.tool_offset_z = QDoubleSpinBox()
        self.tool_offset_z.setRange(-0.5, 0.5)
        self.tool_offset_z.setValue(0.15)
        self.tool_offset_z.setSuffix(' m')
        self.tool_offset_z.setDecimals(3)

        self.btn_set_tool = QPushButton('设置工具偏移')
        self.btn_set_tool.setObjectName("warningButton")
        self.btn_set_tool.clicked.connect(self.set_tool_offset)

        tool_layout.addWidget(QLabel('机器人ID (0:左, 1:右):'), 0, 0)
        tool_layout.addWidget(self.tool_robot_id, 0, 1)
        tool_layout.addWidget(QLabel('X 偏移:'), 1, 0)
        tool_layout.addWidget(self.tool_offset_x, 1, 1)
        tool_layout.addWidget(QLabel('Y 偏移:'), 2, 0)
        tool_layout.addWidget(self.tool_offset_y, 2, 1)
        tool_layout.addWidget(QLabel('Z 偏移:'), 3, 0)
        tool_layout.addWidget(self.tool_offset_z, 3, 1)
        tool_layout.addWidget(self.btn_set_tool, 4, 0, 1, 2)

        tool_group.setLayout(tool_layout)
        layout.addWidget(tool_group)

        layout.addStretch()
        return widget

    def create_servo_tab(self):
        """伺服模式标签页"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # 伺服控制组
        servo_group = QGroupBox('● 伺服模式控制')
        servo_layout = QGridLayout()

        self.btn_start_servo = QPushButton('启动伺服模式')
        self.btn_start_servo.clicked.connect(self.start_servo)
        self.btn_start_servo.setEnabled(False)

        self.btn_stop_servo = QPushButton('停止伺服模式')
        self.btn_stop_servo.setObjectName("stopButton")
        self.btn_stop_servo.clicked.connect(self.stop_servo)
        self.btn_stop_servo.setEnabled(False)

        self.servo_status_label = QLabel('未启动')

        servo_layout.addWidget(QLabel('伺服状态:'), 0, 0)
        servo_layout.addWidget(self.servo_status_label, 0, 1, 1, 2)
        servo_layout.addWidget(self.btn_start_servo, 1, 0)
        servo_layout.addWidget(self.btn_stop_servo, 1, 1)

        servo_group.setLayout(servo_layout)
        layout.addWidget(servo_group)

        # 手动控制提示
        tip_group = QGroupBox('● 使用说明')
        tip_layout = QVBoxLayout()
        tip_label = QLabel(
            '伺服模式启动后，可以：\n'
            '1. 通过VR跟随功能进行实时控制\n'
            '2. 通过ROS话题发送关节或笛卡尔指令\n'
            '  - /jaka/servo/joint_cmd (JakaDualJointServo)\n'
            '  - /jaka/servo/cartesian_cmd (JakaDualCartesianServo)\n'
            '3. 伺服模式需要先上电并使能机器人'
        )
        tip_layout.addWidget(tip_label)
        tip_group.setLayout(tip_layout)
        layout.addWidget(tip_group)

        layout.addStretch()
        return widget

    def create_vr_tab(self):
        """VR跟随标签页"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # VR跟随控制组
        vr_control_group = QGroupBox('● VR跟随控制')
        vr_control_layout = QGridLayout()

        self.btn_enable_vr = QPushButton('启用VR跟随')
        self.btn_enable_vr.setObjectName("infoButton")
        self.btn_enable_vr.clicked.connect(self.toggle_vr_follow)
        self.btn_enable_vr.setEnabled(False)

        self.btn_calibrate_vr = QPushButton('校准VR坐标系')
        self.btn_calibrate_vr.setObjectName("warningButton")
        self.btn_calibrate_vr.clicked.connect(self.calibrate_vr)

        self.vr_status_label = QLabel('未启用')

        vr_control_layout.addWidget(QLabel('VR跟随状态:'), 0, 0)
        vr_control_layout.addWidget(self.vr_status_label, 0, 1, 1, 2)
        vr_control_layout.addWidget(self.btn_enable_vr, 1, 0)
        vr_control_layout.addWidget(self.btn_calibrate_vr, 1, 1)

        vr_control_group.setLayout(vr_control_layout)
        layout.addWidget(vr_control_group)

        # VR控制器信息
        vr_info_group = QGroupBox('● VR控制器实时信息')
        vr_info_layout = QGridLayout()

        self.left_arm_status = QLabel('等待VR数据...')
        self.right_arm_status = QLabel('等待VR数据...')
        self.left_error = QLabel('0.0 mm')
        self.right_error = QLabel('0.0 mm')

        vr_info_layout.addWidget(QLabel('左臂状态:'), 0, 0)
        vr_info_layout.addWidget(self.left_arm_status, 0, 1)
        vr_info_layout.addWidget(QLabel('右臂状态:'), 1, 0)
        vr_info_layout.addWidget(self.right_arm_status, 1, 1)
        vr_info_layout.addWidget(QLabel('左臂跟随误差:'), 2, 0)
        vr_info_layout.addWidget(self.left_error, 2, 1)
        vr_info_layout.addWidget(QLabel('右臂跟随误差:'), 3, 0)
        vr_info_layout.addWidget(self.right_error, 3, 1)

        vr_info_group.setLayout(vr_info_layout)
        layout.addWidget(vr_info_group)

        layout.addStretch()
        return widget

    def create_status_tab(self):
        """状态监控标签页"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # 伺服系统状态组
        servo_status_group = QGroupBox('● 伺服系统状态')
        servo_status_layout = QGridLayout()

        self.mode_label = QLabel('joint')
        self.cycle_time_label = QLabel('8.0 ms')
        self.publish_rate_label = QLabel('125.0 Hz')
        self.latency_label = QLabel('0.0 ms')

        servo_status_layout.addWidget(QLabel('控制模式:'), 0, 0)
        servo_status_layout.addWidget(self.mode_label, 0, 1)
        servo_status_layout.addWidget(QLabel('周期时间:'), 1, 0)
        servo_status_layout.addWidget(self.cycle_time_label, 1, 1)
        servo_status_layout.addWidget(QLabel('发布频率:'), 2, 0)
        servo_status_layout.addWidget(self.publish_rate_label, 2, 1)
        servo_status_layout.addWidget(QLabel('延迟:'), 3, 0)
        servo_status_layout.addWidget(self.latency_label, 3, 1)

        servo_status_group.setLayout(servo_status_layout)
        layout.addWidget(servo_status_group)

        # 机器人状态组
        robot_status_group = QGroupBox('● 机器人状态')
        robot_status_layout = QGridLayout()

        self.robot_powered_label = QLabel('未知')
        self.robot_enabled_label = QLabel('未知')
        self.robot_estop_label = QLabel('未知')
        self.robot_error_label = QLabel('未知')
        self.robot_left_inpos_label = QLabel('未知')
        self.robot_right_inpos_label = QLabel('未知')
        self.robot_error_msg_label = QLabel('无')

        robot_status_layout.addWidget(QLabel('上电状态:'), 0, 0)
        robot_status_layout.addWidget(self.robot_powered_label, 0, 1)
        robot_status_layout.addWidget(QLabel('使能状态:'), 1, 0)
        robot_status_layout.addWidget(self.robot_enabled_label, 1, 1)
        robot_status_layout.addWidget(QLabel('急停状态:'), 2, 0)
        robot_status_layout.addWidget(self.robot_estop_label, 2, 1)
        robot_status_layout.addWidget(QLabel('错误状态:'), 3, 0)
        robot_status_layout.addWidget(self.robot_error_label, 3, 1)
        robot_status_layout.addWidget(QLabel('左臂到位:'), 4, 0)
        robot_status_layout.addWidget(self.robot_left_inpos_label, 4, 1)
        robot_status_layout.addWidget(QLabel('右臂到位:'), 5, 0)
        robot_status_layout.addWidget(self.robot_right_inpos_label, 5, 1)
        robot_status_layout.addWidget(QLabel('错误信息:'), 6, 0)
        robot_status_layout.addWidget(self.robot_error_msg_label, 6, 1)

        robot_status_group.setLayout(robot_status_layout)
        layout.addWidget(robot_status_group)

        # 关节位置组
        joint_group = QGroupBox('● 当前关节位置 (弧度)')
        joint_layout = QGridLayout()

        self.left_joints_labels = []
        self.right_joints_labels = []

        for i in range(7):
            label = QLabel('0.000')
            self.left_joints_labels.append(label)
            joint_layout.addWidget(QLabel(f'左臂关节{i+1}:'), i, 0)
            joint_layout.addWidget(label, i, 1)

        for i in range(7):
            label = QLabel('0.000')
            self.right_joints_labels.append(label)
            joint_layout.addWidget(QLabel(f'右臂关节{i+1}:'), i, 2)
            joint_layout.addWidget(label, i, 3)

        joint_group.setLayout(joint_layout)
        layout.addWidget(joint_group)

        # 笛卡尔位姿组
        cartesian_group = QGroupBox('● 当前末端位姿 (米)')
        cartesian_layout = QGridLayout()

        self.left_pose_x = QLabel('0.000')
        self.left_pose_y = QLabel('0.000')
        self.left_pose_z = QLabel('0.000')
        self.right_pose_x = QLabel('0.000')
        self.right_pose_y = QLabel('0.000')
        self.right_pose_z = QLabel('0.000')

        cartesian_layout.addWidget(QLabel('左臂 X:'), 0, 0)
        cartesian_layout.addWidget(self.left_pose_x, 0, 1)
        cartesian_layout.addWidget(QLabel('左臂 Y:'), 1, 0)
        cartesian_layout.addWidget(self.left_pose_y, 1, 1)
        cartesian_layout.addWidget(QLabel('左臂 Z:'), 2, 0)
        cartesian_layout.addWidget(self.left_pose_z, 2, 1)

        cartesian_layout.addWidget(QLabel('右臂 X:'), 0, 2)
        cartesian_layout.addWidget(self.right_pose_x, 0, 3)
        cartesian_layout.addWidget(QLabel('右臂 Y:'), 1, 2)
        cartesian_layout.addWidget(self.right_pose_y, 1, 3)
        cartesian_layout.addWidget(QLabel('右臂 Z:'), 2, 2)
        cartesian_layout.addWidget(self.right_pose_z, 2, 3)

        cartesian_group.setLayout(cartesian_layout)
        layout.addWidget(cartesian_group)

        layout.addStretch()
        return widget

    def spin_ros(self):
        if rclpy.ok():
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0)
            except Exception:
                pass

    # ========== 基础控制功能 ==========
    def power_on(self):
        req = Trigger.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_power_on, req)
        future.add_done_callback(lambda f: self.on_power_on(f.result()))
        self.add_log("正在上电...", "info")

    def on_power_on(self, response):
        if response.success:
            self.powered_on = True
            self.btn_power_on.setEnabled(False)
            self.btn_power_off.setEnabled(True)
            self.btn_enable.setEnabled(True)
            self.power_status_label.setText('已上电')
            self.add_log("机器人上电成功", "success")
        else:
            self.add_log(f"上电失败: {response.message}", "error")

    def power_off(self):
        req = Trigger.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_power_off, req)
        future.add_done_callback(lambda f: self.on_power_off(f.result()))
        self.add_log("正在下电...", "info")

    def on_power_off(self, response):
        if response.success:
            self.powered_on = False
            self.enabled = False
            self.servo_running = False
            self.btn_power_on.setEnabled(True)
            self.btn_power_off.setEnabled(False)
            self.btn_enable.setEnabled(False)
            self.btn_disable.setEnabled(False)
            self.btn_start_servo.setEnabled(False)
            self.power_status_label.setText('未上电')
            self.enable_status_label.setText('未使能')
            self.add_log("机器人下电成功", "info")
        else:
            self.add_log(f"下电失败: {response.message}", "error")

    def enable_robot(self):
        req = Trigger.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_enable, req)
        future.add_done_callback(lambda f: self.on_enable(f.result()))
        self.add_log("正在使能机器人...", "info")

    def on_enable(self, response):
        if response.success:
            self.enabled = True
            self.btn_enable.setEnabled(False)
            self.btn_disable.setEnabled(True)
            self.btn_start_servo.setEnabled(True)
            self.btn_move_j.setEnabled(True)
            self.btn_move_l.setEnabled(True)
            self.enable_status_label.setText('已使能')
            self.add_log("机器人使能成功", "success")
        else:
            self.add_log(f"使能失败: {response.message}", "error")

    def disable_robot(self):
        req = Trigger.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_disable, req)
        future.add_done_callback(lambda f: self.on_disable(f.result()))
        self.add_log("正在去使能机器人...", "info")

    def on_disable(self, response):
        if response.success:
            self.enabled = False
            self.servo_running = False
            self.btn_enable.setEnabled(True)
            self.btn_disable.setEnabled(False)
            self.btn_start_servo.setEnabled(False)
            self.btn_stop_servo.setEnabled(False)
            self.btn_move_j.setEnabled(False)
            self.btn_move_l.setEnabled(False)
            self.enable_status_label.setText('未使能')
            self.add_log("机器人去使能成功", "info")
        else:
            self.add_log(f"去使能失败: {response.message}", "error")

    def clear_error(self):
        req = Trigger.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_clear_error, req)
        future.add_done_callback(lambda f: self.on_clear_error(f.result()))
        self.add_log("正在清除错误...", "info")

    def on_clear_error(self, response):
        if response.success:
            self.error_status_label.setText('无错误')
            self.add_log("错误清除成功", "success")
        else:
            self.add_log(f"清除错误失败: {response.message}", "error")

    def motion_abort(self):
        req = Trigger.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_motion_abort, req)
        future.add_done_callback(lambda f: self.on_motion_abort(f.result()))
        self.add_log("正在执行急停...", "warning")

    def on_motion_abort(self, response):
        if response.success:
            self.add_log("急停执行成功", "warning")
        else:
            self.add_log(f"急停失败: {response.message}", "error")

    # ========== 伺服模式控制 ==========
    def start_servo(self):
        req = StartServo.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_start_servo, req)
        future.add_done_callback(lambda f: self.on_servo_started(f.result()))
        self.add_log("正在启动伺服模式...", "info")

    def on_servo_started(self, response):
        if response.success:
            self.servo_running = True
            self.btn_start_servo.setEnabled(False)
            self.btn_stop_servo.setEnabled(True)
            self.btn_enable_vr.setEnabled(True)
            self.servo_status_label.setText('已启动')
            self.add_log("伺服模式已启动", "success")
        else:
            self.add_log(f"启动伺服模式失败: {response.message}", "error")

    def stop_servo(self):
        req = StopServo.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_stop_servo, req)
        future.add_done_callback(lambda f: self.on_servo_stopped(f.result()))
        self.add_log("正在停止伺服模式...", "info")

    def on_servo_stopped(self, response):
        if response.success:
            self.servo_running = False
            self.vr_following = False
            self.btn_start_servo.setEnabled(True)
            self.btn_stop_servo.setEnabled(False)
            self.btn_enable_vr.setEnabled(False)
            self.servo_status_label.setText('未启动')
            self.add_log("伺服模式已停止", "info")
        else:
            self.add_log(f"停止伺服模式失败: {response.message}", "error")

    # ========== 点到点运动功能 ==========
    def execute_move_j(self):
        try:
            positions_str = self.movej_positions.text()
            positions = [float(x.strip()) for x in positions_str.split(',')]
            
            if len(positions) != 14:
                self.add_log(f"关节位置数量错误: 需要14个值，实际{len(positions)}个", "error")
                return

            req = MoveJ.Request()
            req.robot_id = int(self.movej_robot_id.value())
            req.joint_positions = positions
            req.move_mode = False  # ABS mode
            req.velocity = self.movej_velocity.value()
            req.acceleration = self.movej_accel.value()
            req.is_block = True

            future = self.ros_node.call_service_async(
                self.ros_node.cli_move_j, req)
            future.add_done_callback(lambda f: self.on_move_j_done(f.result()))
            self.add_log("正在执行MoveJ...", "info")
        except ValueError as e:
            self.add_log(f"输入格式错误: {str(e)}", "error")

    def on_move_j_done(self, response):
        if response.success:
            self.add_log("MoveJ执行成功", "success")
        else:
            self.add_log(f"MoveJ执行失败: {response.message}", "error")

    def execute_move_l(self):
        req = MoveL.Request()
        req.robot_id = int(self.movel_robot_id.value())
        
        # 创建Pose消息
        req.target_pose = Pose()
        req.target_pose.position.x = self.movel_x.value()
        req.target_pose.position.y = self.movel_y.value()
        req.target_pose.position.z = self.movel_z.value()
        req.target_pose.orientation.w = 1.0  # 默认姿态
        
        req.move_mode = False  # ABS mode
        req.velocity = self.movel_velocity.value()
        req.acceleration = self.movel_accel.value()
        req.is_block = True

        future = self.ros_node.call_service_async(
            self.ros_node.cli_move_l, req)
        future.add_done_callback(lambda f: self.on_move_l_done(f.result()))
        self.add_log("正在执行MoveL...", "info")

    def on_move_l_done(self, response):
        if response.success:
            self.add_log("MoveL执行成功", "success")
        else:
            self.add_log(f"MoveL执行失败: {response.message}", "error")

    # ========== 配置功能 ==========
    def set_collision_level(self):
        req = SetCollisionLevel.Request()
        req.robot_id = int(self.collision_robot_id.value())
        req.level = int(self.collision_level.value())

        future = self.ros_node.call_service_async(
            self.ros_node.cli_set_collision, req)
        future.add_done_callback(lambda f: self.on_collision_set(f.result()))
        self.add_log(f"正在设置碰撞等级为{req.level}...", "info")

    def on_collision_set(self, response):
        if response.success:
            self.add_log("碰撞等级设置成功", "success")
        else:
            self.add_log(f"碰撞等级设置失败: {response.message}", "error")

    def set_tool_offset(self):
        req = SetToolOffset.Request()
        req.robot_id = int(self.tool_robot_id.value())
        
        req.tool_offset = Pose()
        req.tool_offset.position.x = self.tool_offset_x.value()
        req.tool_offset.position.y = self.tool_offset_y.value()
        req.tool_offset.position.z = self.tool_offset_z.value()
        req.tool_offset.orientation.w = 1.0

        future = self.ros_node.call_service_async(
            self.ros_node.cli_set_tool, req)
        future.add_done_callback(lambda f: self.on_tool_set(f.result()))
        self.add_log("正在设置工具偏移...", "info")

    def on_tool_set(self, response):
        if response.success:
            self.add_log("工具偏移设置成功", "success")
        else:
            self.add_log(f"工具偏移设置失败: {response.message}", "error")

    # ========== VR跟随功能 ==========
    def toggle_vr_follow(self):
        req = EnableVRFollow.Request()
        req.enable = not self.vr_following
        future = self.ros_node.call_service_async(
            self.ros_node.cli_enable_vr, req)
        future.add_done_callback(lambda f: self.on_vr_toggled(f.result()))

        action = "启用" if not self.vr_following else "停止"
        self.add_log(f"正在{action}VR跟随...", "info")

    def on_vr_toggled(self, response):
        if response.success:
            self.vr_following = not self.vr_following
            if self.vr_following:
                self.btn_enable_vr.setText('停止VR跟随')
                self.vr_status_label.setText('已启用')
                self.add_log("VR跟随已启用", "success")
            else:
                self.btn_enable_vr.setText('启用VR跟随')
                self.vr_status_label.setText('未启用')
                self.add_log("VR跟随已停止", "info")
        else:
            self.add_log(f"VR跟随操作失败: {response.message}", "error")

    def calibrate_vr(self):
        req = CalibrateVR.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_calibrate_vr, req)
        future.add_done_callback(lambda f: self.on_vr_calibrated(f.result()))
        self.add_log("正在校准VR坐标系...", "info")

    def on_vr_calibrated(self, response):
        if response.success:
            self.add_log("VR坐标系校准成功", "success")
        else:
            self.add_log(f"VR校准失败: {response.message}", "error")

    # ========== 状态更新 ==========
    def update_status(self, msg):
        self.mode_label.setText(msg.mode)
        self.cycle_time_label.setText(f'{msg.cycle_time_ns / 1e6:.2f} ms')
        self.publish_rate_label.setText(f'{msg.publish_rate_hz:.1f} Hz')
        self.latency_label.setText(f'{msg.latency_ms:.2f} ms')

    def update_vr_status(self, msg):
        self.left_arm_status.setText(msg.left_arm_status)
        self.right_arm_status.setText(msg.right_arm_status)
        self.left_error.setText(f'{msg.left_pose_error:.2f} mm')
        self.right_error.setText(f'{msg.right_pose_error:.2f} mm')

    def update_robot_state(self, msg):
        """更新机器人状态显示"""
        self.robot_powered_label.setText('已上电' if msg.powered_on else '未上电')
        self.robot_enabled_label.setText('已使能' if msg.servo_enabled else '未使能')
        self.robot_estop_label.setText('急停中' if msg.estoped else '正常')
        self.robot_error_label.setText('错误' if msg.in_error else '正常')
        self.robot_left_inpos_label.setText('到位' if msg.left_in_position else '运动中')
        self.robot_right_inpos_label.setText('到位' if msg.right_in_position else '运动中')
        self.robot_error_msg_label.setText(msg.error_message if msg.error_message else '无')

        # 更新关节位置
        if len(msg.left_joint_positions) == 7:
            for i, pos in enumerate(msg.left_joint_positions):
                self.left_joints_labels[i].setText(f'{pos:.3f}')
        
        if len(msg.right_joint_positions) == 7:
            for i, pos in enumerate(msg.right_joint_positions):
                self.right_joints_labels[i].setText(f'{pos:.3f}')

        # 更新笛卡尔位姿
        self.left_pose_x.setText(f'{msg.left_cartesian_pose.position.x:.3f}')
        self.left_pose_y.setText(f'{msg.left_cartesian_pose.position.y:.3f}')
        self.left_pose_z.setText(f'{msg.left_cartesian_pose.position.z:.3f}')
        self.right_pose_x.setText(f'{msg.right_cartesian_pose.position.x:.3f}')
        self.right_pose_y.setText(f'{msg.right_cartesian_pose.position.y:.3f}')
        self.right_pose_z.setText(f'{msg.right_cartesian_pose.position.z:.3f}')

    def update_joint_state(self, msg):
        """更新关节状态（用于备用）"""
        pass  # 已在 update_robot_state 中处理

    def add_log(self, message, level="info"):
        colors = {
            "info": "black",
            "success": "green",
            "warning": "orange",
            "error": "red"
        }
        color = colors.get(level, "black")
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(
            f'<span style="color: {color};">'
            f'[{timestamp}] {message}</span>'
        )
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum())

    def closeEvent(self, event):
        self.ros_timer.stop()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    # 允许 Ctrl+C 退出
    signal.signal(signal.SIGINT, lambda *args: app.quit())

    window = JakaControlGUI()
    window.show()

    exit_code = app.exec_()

    if rclpy.ok():
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
