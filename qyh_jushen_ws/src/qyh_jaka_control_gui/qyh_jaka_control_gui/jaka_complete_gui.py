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

from qyh_jaka_control_msgs.msg import (
    JakaDualJointServo,
    JakaDualCartesianServo,
    JakaServoStatus,
    VRFollowStatus
)
from qyh_jaka_control_msgs.srv import (
    StartServo, StopServo,
    EnableVRFollow, CalibrateVR,
    StartRecording, StopRecording
)


class SignalBridge(QObject):
    """线程安全的信号桥接"""
    status_updated = pyqtSignal(object)
    vr_status_updated = pyqtSignal(object)
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
        self.cli_start_recording = self.create_client(
            StartRecording, '/jaka/vr/start_recording')
        self.cli_stop_recording = self.create_client(
            StopRecording, '/jaka/vr/stop_recording')

    def status_callback(self, msg):
        self.signals.status_updated.emit(msg)

    def vr_status_callback(self, msg):
        self.signals.vr_status_updated.emit(msg)

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
        self.recording = False

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
        tabs.addTab(self.create_servo_tab(), "伺服模式")
        tabs.addTab(self.create_vr_tab(), "VR跟随")
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

        # 数据录制组
        record_group = QGroupBox('● 数据录制（用于具身智能训练）')
        record_layout = QGridLayout()

        self.btn_start_recording = QPushButton('开始录制')
        self.btn_start_recording.setObjectName("warningButton")
        self.btn_start_recording.clicked.connect(self.start_recording)
        self.btn_start_recording.setEnabled(False)

        self.btn_stop_recording = QPushButton('停止录制')
        self.btn_stop_recording.setObjectName("stopButton")
        self.btn_stop_recording.clicked.connect(self.stop_recording)
        self.btn_stop_recording.setEnabled(False)

        self.output_path = QLineEdit('/tmp/jaka_recordings')
        self.recording_freq = QDoubleSpinBox()
        self.recording_freq.setRange(1.0, 125.0)
        self.recording_freq.setValue(125.0)
        self.recording_freq.setSuffix(' Hz')

        self.recording_status = QLabel('未录制')
        self.frames_label = QLabel('已录制帧数: 0')
        self.duration_label = QLabel('录制时长: 0.0 秒')

        record_layout.addWidget(QLabel('输出路径:'), 0, 0)
        record_layout.addWidget(self.output_path, 0, 1, 1, 2)
        record_layout.addWidget(QLabel('录制频率:'), 1, 0)
        record_layout.addWidget(self.recording_freq, 1, 1, 1, 2)
        record_layout.addWidget(QLabel('录制状态:'), 2, 0)
        record_layout.addWidget(self.recording_status, 2, 1, 1, 2)
        record_layout.addWidget(self.btn_start_recording, 3, 0)
        record_layout.addWidget(self.btn_stop_recording, 3, 1)
        record_layout.addWidget(self.frames_label, 4, 0, 1, 2)
        record_layout.addWidget(self.duration_label, 5, 0, 1, 2)

        record_group.setLayout(record_layout)
        layout.addWidget(record_group)

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

        # 系统状态组
        status_group = QGroupBox('● 实时系统状态')
        status_layout = QGridLayout()

        self.mode_label = QLabel('joint')
        self.cycle_time_label = QLabel('8.0 ms')
        self.publish_rate_label = QLabel('125.0 Hz')
        self.latency_label = QLabel('0.0 ms')

        status_layout.addWidget(QLabel('控制模式:'), 0, 0)
        status_layout.addWidget(self.mode_label, 0, 1)
        status_layout.addWidget(QLabel('周期时间:'), 1, 0)
        status_layout.addWidget(self.cycle_time_label, 1, 1)
        status_layout.addWidget(QLabel('发布频率:'), 2, 0)
        status_layout.addWidget(self.publish_rate_label, 2, 1)
        status_layout.addWidget(QLabel('延迟:'), 3, 0)
        status_layout.addWidget(self.latency_label, 3, 1)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

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
            self.recording = False
            self.btn_start_servo.setEnabled(True)
            self.btn_stop_servo.setEnabled(False)
            self.btn_enable_vr.setEnabled(False)
            self.btn_start_recording.setEnabled(False)
            self.btn_stop_recording.setEnabled(False)
            self.servo_status_label.setText('未启动')
            self.add_log("伺服模式已停止", "info")
        else:
            self.add_log(f"停止伺服模式失败: {response.message}", "error")

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
                self.btn_start_recording.setEnabled(True)
                self.vr_status_label.setText('已启用')
                self.add_log("VR跟随已启用", "success")
            else:
                self.btn_enable_vr.setText('启用VR跟随')
                self.btn_start_recording.setEnabled(False)
                self.btn_stop_recording.setEnabled(False)
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

    def start_recording(self):
        req = StartRecording.Request()
        req.output_path = self.output_path.text()
        req.recording_frequency = self.recording_freq.value()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_start_recording, req)
        future.add_done_callback(
            lambda f: self.on_recording_started(f.result()))
        self.add_log("正在开始录制...", "info")

    def on_recording_started(self, response):
        if response.success:
            self.recording = True
            self.btn_start_recording.setEnabled(False)
            self.btn_stop_recording.setEnabled(True)
            self.recording_status.setText('录制中')
            self.add_log(f"录制已开始: {response.message}", "success")
        else:
            self.add_log(f"开始录制失败: {response.message}", "error")

    def stop_recording(self):
        req = StopRecording.Request()
        future = self.ros_node.call_service_async(
            self.ros_node.cli_stop_recording, req)
        future.add_done_callback(
            lambda f: self.on_recording_stopped(f.result()))
        self.add_log("正在停止录制...", "info")

    def on_recording_stopped(self, response):
        if response.success:
            self.recording = False
            self.btn_start_recording.setEnabled(True)
            self.btn_stop_recording.setEnabled(False)
            self.recording_status.setText('未录制')
            self.add_log(
                f"录制已停止: 共{response.total_frames}帧, "
                f"{response.duration:.2f}秒, 保存至{response.saved_file}",
                "success"
            )
        else:
            self.add_log(f"停止录制失败: {response.message}", "error")

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
        self.frames_label.setText(f'已录制帧数: {msg.recorded_frames}')
        self.duration_label.setText(
            f'录制时长: {msg.recording_duration:.1f} 秒')

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
