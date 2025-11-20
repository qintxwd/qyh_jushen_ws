#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import signal
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QGroupBox, QGridLayout, QPushButton, QCheckBox, 
                             QDoubleSpinBox, QLineEdit, QTextEdit, QTabWidget, QProgressBar)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor

from qyh_jaka_control_msgs.msg import JakaDualJointServo, JakaDualCartesianServo, JakaServoStatus, VRFollowStatus
from qyh_jaka_control_msgs.srv import StartServo, StopServo, EnableVRFollow, StartRecording, StopRecording, CalibrateVR


class SignalBridge(QObject):
    """线程安全的信号桥接"""
    status_updated = pyqtSignal(object)
    vr_status_updated = pyqtSignal(object)
    log_message = pyqtSignal(str, str)  # message, level


class JakaControlNode(Node):
    def __init__(self, signals):
        super().__init__('jaka_control_gui')
        self.signals = signals
        
        # Publishers
        self.joint_pub = self.create_publisher(JakaDualJointServo, '/jaka/servo/joint_cmd', 1)
        self.cart_pub = self.create_publisher(JakaDualCartesianServo, '/jaka/servo/cartesian_cmd', 1)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            JakaServoStatus, '/jaka/servo/status', self.status_callback, 10)
        self.vr_status_sub = self.create_subscription(
            VRFollowStatus, '/jaka/vr/status', self.vr_status_callback, 10)
        
        # Service clients
        self.cli_start_servo = self.create_client(StartServo, '/jaka/servo/start')
        self.cli_stop_servo = self.create_client(StopServo, '/jaka/servo/stop')
        self.cli_enable_vr = self.create_client(EnableVRFollow, '/jaka/vr/enable')
        self.cli_start_recording = self.create_client(StartRecording, '/jaka/vr/start_recording')
        self.cli_stop_recording = self.create_client(StopRecording, '/jaka/vr/stop_recording')
        self.cli_calibrate_vr = self.create_client(CalibrateVR, '/jaka/vr/calibrate')
    
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
        self.setWindowTitle('JAKA双臂VR控制系统')
        self.setGeometry(100, 100, 1200, 800)
        
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
        self.servo_running = False
        self.vr_following = False
        self.recording = False
        
        self.add_log("JAKA双臂VR控制系统已启动", "info")
    
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
                padding: 0 8px 0 8px;
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
            QPushButton#vrButton {
                background-color: #2196F3;
            }
            QPushButton#vrButton:hover {
                background-color: #0b7dda;
            }
            QPushButton#recordButton {
                background-color: #FF9800;
            }
            QPushButton#recordButton:hover {
                background-color: #e68900;
            }
        """)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(15, 15, 15, 15)
        
        # 标题
        header = QLabel('◆ JAKA双臂VR控制系统 ◆')
        header_font = QFont()
        header_font.setPointSize(20)
        header_font.setBold(True)
        header.setFont(header_font)
        header.setStyleSheet('color: #2c5aa0; background: transparent;')
        header.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(header)
        
        # 标签页
        tabs = QTabWidget()
        tabs.addTab(self.create_control_tab(), "伺服控制")
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
    
    def create_control_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # 伺服控制组
        servo_group = QGroupBox('● 伺服模式控制')
        servo_layout = QGridLayout()
        
        self.btn_start_servo = QPushButton('启动伺服模式')
        self.btn_start_servo.clicked.connect(self.start_servo)
        
        self.btn_stop_servo = QPushButton('停止伺服模式')
        self.btn_stop_servo.setObjectName("stopButton")
        self.btn_stop_servo.clicked.connect(self.stop_servo)
        self.btn_stop_servo.setEnabled(False)
        
        self.servo_status_label = QLabel('未启动')
        self.servo_status_label.setStyleSheet('''
            background-color: #f8f9fa; padding: 8px 12px; 
            border: 1px solid #dee2e6; border-radius: 5px;
        ''')
        
        servo_layout.addWidget(QLabel('伺服状态:'), 0, 0)
        servo_layout.addWidget(self.servo_status_label, 0, 1, 1, 2)
        servo_layout.addWidget(self.btn_start_servo, 1, 0)
        servo_layout.addWidget(self.btn_stop_servo, 1, 1)
        
        servo_group.setLayout(servo_layout)
        layout.addWidget(servo_group)
        
        # 手动控制组（简化版，主要用于测试）
        manual_group = QGroupBox('● 手动测试控制')
        manual_layout = QGridLayout()
        
        manual_layout.addWidget(QLabel('提示: 请使用VR跟随功能进行实际控制'), 0, 0, 1, 2)
        manual_layout.addWidget(QLabel('此区域预留给手动测试功能'), 1, 0, 1, 2)
        
        manual_group.setLayout(manual_layout)
        layout.addWidget(manual_group)
        
        layout.addStretch()
        return widget
    
    def create_vr_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        # VR跟随控制组
        vr_control_group = QGroupBox('● VR跟随控制')
        vr_control_layout = QGridLayout()
        
        self.btn_enable_vr = QPushButton('启用VR跟随')
        self.btn_enable_vr.setObjectName("vrButton")
        self.btn_enable_vr.clicked.connect(self.toggle_vr_follow)
        self.btn_enable_vr.setEnabled(False)
        
        self.btn_calibrate_vr = QPushButton('校准VR坐标系')
        self.btn_calibrate_vr.clicked.connect(self.calibrate_vr)
        
        self.vr_status_label = QLabel('未启用')
        self.vr_status_label.setStyleSheet('''
            background-color: #f8f9fa; padding: 8px 12px;
            border: 1px solid #dee2e6; border-radius: 5px;
        ''')
        
        vr_control_layout.addWidget(QLabel('VR跟随状态:'), 0, 0)
        vr_control_layout.addWidget(self.vr_status_label, 0, 1, 1, 2)
        vr_control_layout.addWidget(self.btn_enable_vr, 1, 0)
        vr_control_layout.addWidget(self.btn_calibrate_vr, 1, 1)
        
        vr_control_group.setLayout(vr_control_layout)
        layout.addWidget(vr_control_group)
        
        # 数据录制组
        record_group = QGroupBox('● 数据录制')
        record_layout = QGridLayout()
        
        self.btn_start_recording = QPushButton('开始录制')
        self.btn_start_recording.setObjectName("recordButton")
        self.btn_start_recording.clicked.connect(self.start_recording)
        self.btn_start_recording.setEnabled(False)
        
        self.btn_stop_recording = QPushButton('停止录制')
        self.btn_stop_recording.setObjectName("stopButton")
        self.btn_stop_recording.clicked.connect(self.stop_recording)
        self.btn_stop_recording.setEnabled(False)
        
        self.output_path = QLineEdit('/tmp/jaka_recordings')
        self.output_path.setPlaceholderText('录制数据输出路径')
        
        self.recording_status = QLabel('未录制')
        self.recording_status.setStyleSheet('''
            background-color: #f8f9fa; padding: 8px 12px;
            border: 1px solid #dee2e6; border-radius: 5px;
        ''')
        
        self.frames_label = QLabel('已录制帧数: 0')
        self.duration_label = QLabel('录制时长: 0.0 秒')
        
        record_layout.addWidget(QLabel('输出路径:'), 0, 0)
        record_layout.addWidget(self.output_path, 0, 1, 1, 2)
        record_layout.addWidget(QLabel('录制状态:'), 1, 0)
        record_layout.addWidget(self.recording_status, 1, 1, 1, 2)
        record_layout.addWidget(self.btn_start_recording, 2, 0)
        record_layout.addWidget(self.btn_stop_recording, 2, 1)
        record_layout.addWidget(self.frames_label, 3, 0, 1, 2)
        record_layout.addWidget(self.duration_label, 4, 0, 1, 2)
        
        record_group.setLayout(record_layout)
        layout.addWidget(record_group)
        
        # VR信息显示
        vr_info_group = QGroupBox('● VR控制器信息')
        vr_info_layout = QGridLayout()
        
        self.left_arm_status = QLabel('等待连接...')
        self.right_arm_status = QLabel('等待连接...')
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
        widget = QWidget()
        layout = QVBoxLayout(widget)
        
        status_group = QGroupBox('● 系统状态')
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
    
    def start_servo(self):
        req = StartServo.Request()
        future = self.ros_node.call_service_async(self.ros_node.cli_start_servo, req)
        future.add_done_callback(lambda f: self.on_servo_started(f.result()))
        self.add_log("正在启动伺服模式...", "info")
    
    def on_servo_started(self, response):
        if response.success:
            self.servo_running = True
            self.btn_start_servo.setEnabled(False)
            self.btn_stop_servo.setEnabled(True)
            self.btn_enable_vr.setEnabled(True)
            self.servo_status_label.setText('已启动')
            self.servo_status_label.setStyleSheet('''
                background-color: #c8e6c9; padding: 8px 12px;
                border: 1px solid #4caf50; border-radius: 5px; color: #2e7d32;
            ''')
            self.add_log("伺服模式已启动", "success")
        else:
            self.add_log(f"启动伺服模式失败: {response.message}", "error")
    
    def stop_servo(self):
        req = StopServo.Request()
        future = self.ros_node.call_service_async(self.ros_node.cli_stop_servo, req)
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
            self.servo_status_label.setStyleSheet('''
                background-color: #f8f9fa; padding: 8px 12px;
                border: 1px solid #dee2e6; border-radius: 5px;
            ''')
            self.add_log("伺服模式已停止", "info")
        else:
            self.add_log(f"停止伺服模式失败: {response.message}", "error")
    
    def toggle_vr_follow(self):
        req = EnableVRFollow.Request()
        req.enable = not self.vr_following
        future = self.ros_node.call_service_async(self.ros_node.cli_enable_vr, req)
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
                self.vr_status_label.setStyleSheet('''
                    background-color: #c8e6c9; padding: 8px 12px;
                    border: 1px solid #4caf50; border-radius: 5px; color: #2e7d32;
                ''')
                self.add_log("VR跟随已启用", "success")
            else:
                self.btn_enable_vr.setText('启用VR跟随')
                self.btn_start_recording.setEnabled(False)
                self.btn_stop_recording.setEnabled(False)
                self.vr_status_label.setText('未启用')
                self.vr_status_label.setStyleSheet('''
                    background-color: #f8f9fa; padding: 8px 12px;
                    border: 1px solid #dee2e6; border-radius: 5px;
                ''')
                self.add_log("VR跟随已停止", "info")
        else:
            self.add_log(f"VR跟随操作失败: {response.message}", "error")
    
    def start_recording(self):
        req = StartRecording.Request()
        req.output_path = self.output_path.text()
        req.recording_frequency = 125.0
        future = self.ros_node.call_service_async(self.ros_node.cli_start_recording, req)
        future.add_done_callback(lambda f: self.on_recording_started(f.result()))
        self.add_log("正在开始录制...", "info")
    
    def on_recording_started(self, response):
        if response.success:
            self.recording = True
            self.btn_start_recording.setEnabled(False)
            self.btn_stop_recording.setEnabled(True)
            self.recording_status.setText('录制中')
            self.recording_status.setStyleSheet('''
                background-color: #fff9c4; padding: 8px 12px;
                border: 1px solid #fbc02d; border-radius: 5px; color: #f57f17;
            ''')
            self.add_log(f"录制已开始: {response.message}", "success")
        else:
            self.add_log(f"开始录制失败: {response.message}", "error")
    
    def stop_recording(self):
        req = StopRecording.Request()
        future = self.ros_node.call_service_async(self.ros_node.cli_stop_recording, req)
        future.add_done_callback(lambda f: self.on_recording_stopped(f.result()))
        self.add_log("正在停止录制...", "info")
    
    def on_recording_stopped(self, response):
        if response.success:
            self.recording = False
            self.btn_start_recording.setEnabled(True)
            self.btn_stop_recording.setEnabled(False)
            self.recording_status.setText('未录制')
            self.recording_status.setStyleSheet('''
                background-color: #f8f9fa; padding: 8px 12px;
                border: 1px solid #dee2e6; border-radius: 5px;
            ''')
            self.add_log(f"录制已停止: 共{response.total_frames}帧, "
                        f"{response.duration:.2f}秒, 保存至{response.saved_file}", "success")
        else:
            self.add_log(f"停止录制失败: {response.message}", "error")
    
    def calibrate_vr(self):
        # 简单实现：可以扩展为弹出对话框让用户输入校准参数
        self.add_log("VR坐标系校准功能待实现", "warning")
    
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
        self.duration_label.setText(f'录制时长: {msg.recording_duration:.1f} 秒')
    
    def add_log(self, message, level="info"):
        colors = {
            "info": "black",
            "success": "green",
            "warning": "orange",
            "error": "red"
        }
        color = colors.get(level, "black")
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_text.append(f'<span style="color: {color};">[{timestamp}] {message}</span>')
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
