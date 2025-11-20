#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import rclpy
from rclpy.node import Node
from qyh_standard_robot_msgs.msg import StandardRobotStatus
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QGroupBox, 
                             QGridLayout, QProgressBar, QScrollArea, QPushButton, QCheckBox)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QObject, QEvent, QPointF
from PyQt5.QtGui import QFont, QPalette, QColor, QPainter
from qyh_standard_robot_msgs.msg import ManualMotionCommand
from qyh_standard_robot_msgs.srv import (
    ControlStartManualControl,
    ControlStopManualControl,
    ControlPauseMove,
    ControlResumeMove,
    ControlStopMove,
    ControlStopLocalization,
    ControlEmergencyStop,
    ControlReleaseEmergencyStop,
    ControlStartCharging,
    ControlStopCharging,
    ControlEnterLowPowerMode,
    ControlExitLowPowerMode,
    ControlSystemReset,
    ControlPauseMission,
    ControlResumeMission,
    ControlCancelMission,
)


class RobotStatusSignals(QObject):
    """信号类，用于线程安全的UI更新"""
    status_updated = pyqtSignal(object)


class ToggleSwitch(QCheckBox):
    """自定义开关控件，模拟 Android 风格"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCursor(Qt.PointingHandCursor)
        self.setFixedSize(62, 32)
        self.setFocusPolicy(Qt.NoFocus)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect()
        radius = rect.height() / 2
        track_color = QColor('#4CAF50') if self.isChecked() else QColor('#cfd8dc')
        knob_color = QColor('#ffffff') if self.isEnabled() else QColor('#f0f0f0')
        border_color = QColor('#2E7D32') if self.isChecked() else QColor('#b0bec5')

        painter.setPen(border_color)
        painter.setBrush(track_color)
        painter.drawRoundedRect(rect.adjusted(1, 1, -1, -1), radius, radius)

        knob_radius = radius - 4
        center_x = rect.right() - radius if self.isChecked() else rect.left() + radius
        knob_center = QPointF(center_x, rect.center().y())
        painter.setPen(Qt.NoPen)
        painter.setBrush(knob_color)
        painter.drawEllipse(knob_center, knob_radius, knob_radius)


class StandardRobotMonitor(Node):
    def __init__(self, signals):
        super().__init__('standard_robot_monitor')
        self.signals = signals
        self.latest_status = None
        self.manual_pub = self.create_publisher(ManualMotionCommand, 'manual_motion_cmd', 10)
        self.cli_start_manual = self.create_client(ControlStartManualControl, 'control_start_manual_control')
        self.cli_stop_manual = self.create_client(ControlStopManualControl, 'control_stop_manual_control')
        self.cli_pause_move = self.create_client(ControlPauseMove, 'control_pause_move')
        self.cli_resume_move = self.create_client(ControlResumeMove, 'control_resume_move')
        self.cli_stop_move = self.create_client(ControlStopMove, 'control_stop_move')
        self.cli_stop_localization = self.create_client(ControlStopLocalization, 'control_stop_localization')
        self.cli_emergency_stop = self.create_client(ControlEmergencyStop, 'control_emergency_stop')
        self.cli_release_emergency_stop = self.create_client(ControlReleaseEmergencyStop, 'control_release_emergency_stop')
        self.cli_start_charging = self.create_client(ControlStartCharging, 'control_start_charging')
        self.cli_stop_charging = self.create_client(ControlStopCharging, 'control_stop_charging')
        self.cli_enter_low_power = self.create_client(ControlEnterLowPowerMode, 'control_enter_low_power_mode')
        self.cli_exit_low_power = self.create_client(ControlExitLowPowerMode, 'control_exit_low_power_mode')
        self.cli_system_reset = self.create_client(ControlSystemReset, 'control_system_reset')
        self.cli_pause_mission = self.create_client(ControlPauseMission, 'control_pause_mission')
        self.cli_resume_mission = self.create_client(ControlResumeMission, 'control_resume_mission')
        self.cli_cancel_mission = self.create_client(ControlCancelMission, 'control_cancel_mission')
        
        # 订阅机器人状态
        self.subscription = self.create_subscription(
            StandardRobotStatus,
            'standard_robot_status',
            self.status_callback,
            10)
        
        self.get_logger().info('Standard Robot Monitor started')
    
    def status_callback(self, msg):
        self.latest_status = msg
        self.signals.status_updated.emit(msg)
    def publish_manual(self, cmd):
        self.manual_pub.publish(cmd)
    def call_service(self, client, request):
        if client.service_is_ready():
            return client.call_async(request)
        return None


class RobotMonitorGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Standard Robot Monitor')
        self.setGeometry(100, 100, 1400, 900)
        
        # 设置优化的样式
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
                background-color: white;
            }
            QLabel {
                padding: 3px;
                color: #333;
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
            QScrollArea {
                border: none;
                background-color: transparent;
            }
            QGroupBox#controlServices QPushButton {
                background-color: #1e88e5;
                border: 2px solid #1565c0;
                border-radius: 8px;
                padding: 10px 14px;
                font-size: 10pt;
                font-weight: 600;
                color: #ffffff;
            }
            QGroupBox#controlServices QPushButton:hover {
                background-color: #1976d2;
            }
            QGroupBox#controlServices QPushButton:pressed {
                background-color: #0d47a1;
            }
        """)
        
        # 初始化ROS2
        self.signals = RobotStatusSignals()
        self.signals.status_updated.connect(self.update_status)
        
        rclpy.init()
        self.ros_node = StandardRobotMonitor(self.signals)
        
        # 创建定时器来处理ROS2回调
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.ros_node, timeout_sec=0))
        self.ros_timer.start(50)  # 20Hz
        
        # 创建UI
        self.init_ui()
        
        # 连接状态标签
        self.last_update_time = None
        self.manual_enabled = False
        self.cmd = ManualMotionCommand()
        self.cmd.forward = False
        self.cmd.backward = False
        self.cmd.rotate_left = False
        self.cmd.rotate_right = False
        self.installEventFilter(self)
        self.manual_timer = QTimer()
        self.manual_timer.timeout.connect(self.publish_manual_cmd)
        self.manual_timer.start(50)
        QApplication.instance().installEventFilter(self)
    
    def init_ui(self):
        # 创建中央widget和滚动区域
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)
        main_layout.setContentsMargins(15, 15, 15, 15)
        
        # 标题栏
        header_widget = QWidget()
        header_widget.setStyleSheet("""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                      stop:0 #2c5aa0, stop:1 #4a90e2);
            border-radius: 10px;
            padding: 15px;
        """)
        header_layout = QHBoxLayout(header_widget)
        
        # 标题
        title_label = QLabel('◆ 机器人状态实时监控系统 ◆')
        title_font = QFont()
        title_font.setPointSize(20)
        title_font.setBold(True)
        title_label.setFont(title_font)
        title_label.setStyleSheet('color: white; background: transparent;')
        header_layout.addWidget(title_label)
        
        header_layout.addStretch()
        
        # 连接状态
        self.connection_label = QLabel('● 等待连接...')
        self.connection_label.setFont(QFont('', 11, QFont.Bold))
        self.connection_label.setStyleSheet("""
            background-color: #ff9800; 
            color: white; 
            padding: 8px 15px; 
            border-radius: 5px;
        """)
        header_layout.addWidget(self.connection_label)
        
        main_layout.addWidget(header_widget)
        
        # 创建滚动区域
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setFrameShape(QScrollArea.NoFrame)
        
        # 滚动区域内容widget
        scroll_content = QWidget()
        scroll_content.setStyleSheet('background-color: transparent;')
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setSpacing(15)
        scroll_layout.setContentsMargins(5, 5, 5, 5)
        
        # 创建所有内容组
        self.create_all_content(scroll_layout)
        
        scroll_area.setWidget(scroll_content)
        main_layout.addWidget(scroll_area)
        self.setFocusPolicy(Qt.StrongFocus)
        central_widget.setFocusPolicy(Qt.StrongFocus)
    
    def create_all_content(self, parent_layout):
        """创建所有内容在一个页面"""
        # 第一行：系统状态和位姿信息
        row1 = QHBoxLayout()
        row1.addWidget(self.create_system_status_group())
        row1.addWidget(self.create_pose_group())
        parent_layout.addLayout(row1)
        
        # 第二行：速度信息、电池状态和统计信息
        row2 = QHBoxLayout()
        row2.addWidget(self.create_velocity_group(), 2)
        row2.addWidget(self.create_battery_group(), 3)
        row2.addWidget(self.create_statistics_group(), 2)
        parent_layout.addLayout(row2)
        
        # 第三行：任务信息
        row3 = QHBoxLayout()
        row3.addWidget(self.create_mission_group())
        row3.addWidget(self.create_move_task_group())
        parent_layout.addLayout(row3)
        
        # 第四行：状态标志
        row4 = QHBoxLayout()
        row4.addWidget(self.create_status_flags_group())
        row4.addWidget(self.create_obstacle_flags_group())
        parent_layout.addLayout(row4)
        row5 = QHBoxLayout()
        row5.addWidget(self.create_manual_control_group(), 2)
        row5.addWidget(self.create_control_services_group(), 3)
        parent_layout.addLayout(row5)
        
        parent_layout.addStretch()
    
    def create_system_status_group(self):
        """创建系统状态组"""
        group = QGroupBox('▣ 系统状态')
        grid = QGridLayout()
        
        self.sys_status_label = self.create_value_label('未知')
        self.location_status_label = self.create_value_label('未知')
        self.operation_status_label = self.create_value_label('未知')
        self.scheduling_mode_label = self.create_value_label('未知')
        self.motion_status_label = self.create_value_label('未知')
        self.station_label = self.create_value_label('-')
        self.error_code_label = self.create_value_label('0x00000000')
        self.volume_label = self.create_value_label('0')
        self.ip_label = self.create_value_label('0.0.0.0')
        self.map_label = self.create_value_label('-')
        
        grid.addWidget(QLabel('系统状态:'), 0, 0)
        grid.addWidget(self.sys_status_label, 0, 1)
        grid.addWidget(QLabel('定位状态:'), 1, 0)
        grid.addWidget(self.location_status_label, 1, 1)
        grid.addWidget(QLabel('操作状态:'), 2, 0)
        grid.addWidget(self.operation_status_label, 2, 1)
        grid.addWidget(QLabel('调度模式:'), 3, 0)
        grid.addWidget(self.scheduling_mode_label, 3, 1)
        grid.addWidget(QLabel('运动状态:'), 4, 0)
        grid.addWidget(self.motion_status_label, 4, 1)
        grid.addWidget(QLabel('当前站点:'), 5, 0)
        grid.addWidget(self.station_label, 5, 1)
        grid.addWidget(QLabel('最后错误码:'), 6, 0)
        grid.addWidget(self.error_code_label, 6, 1)
        grid.addWidget(QLabel('系统音量:'), 7, 0)
        grid.addWidget(self.volume_label, 7, 1)
        grid.addWidget(QLabel('IP地址:'), 8, 0)
        grid.addWidget(self.ip_label, 8, 1)
        grid.addWidget(QLabel('当前地图:'), 9, 0)
        grid.addWidget(self.map_label, 9, 1)
        
        group.setLayout(grid)
        return group
    
    def create_pose_group(self):
        """创建位姿信息组"""
        group = QGroupBox('※ 位姿信息')
        grid = QGridLayout()
        
        self.pos_x_label = self.create_value_label('0.00 m')
        self.pos_y_label = self.create_value_label('0.00 m')
        self.yaw_label = self.create_value_label('0.00 rad')
        self.confidence_label = self.create_value_label('0.0%')
        
        grid.addWidget(QLabel('X 坐标:'), 0, 0)
        grid.addWidget(self.pos_x_label, 0, 1)
        grid.addWidget(QLabel('Y 坐标:'), 1, 0)
        grid.addWidget(self.pos_y_label, 1, 1)
        grid.addWidget(QLabel('航向角:'), 2, 0)
        grid.addWidget(self.yaw_label, 2, 1)
        grid.addWidget(QLabel('位姿置信度:'), 3, 0)
        grid.addWidget(self.confidence_label, 3, 1)
        
        group.setLayout(grid)
        return group
    
    def create_velocity_group(self):
        """创建速度信息组"""
        group = QGroupBox('▶ 速度信息')
        grid = QGridLayout()
        
        self.vel_x_label = self.create_value_label('0.00 m/s')
        self.vel_y_label = self.create_value_label('0.00 m/s')
        self.omega_label = self.create_value_label('0.00 rad/s')
        
        grid.addWidget(QLabel('X 方向速度:'), 0, 0)
        grid.addWidget(self.vel_x_label, 0, 1)
        grid.addWidget(QLabel('Y 方向速度:'), 1, 0)
        grid.addWidget(self.vel_y_label, 1, 1)
        grid.addWidget(QLabel('角速度:'), 2, 0)
        grid.addWidget(self.omega_label, 2, 1)
        
        group.setLayout(grid)
        return group
    
    def create_battery_group(self):
        """创建电池状态组"""
        group = QGroupBox('◈ 电池状态')
        layout = QVBoxLayout()
        layout.setSpacing(10)
        
        # 电量进度条容器
        progress_container = QWidget()
        progress_layout = QVBoxLayout(progress_container)
        progress_layout.setContentsMargins(5, 5, 5, 5)
        
        self.battery_progress = QProgressBar()
        self.battery_progress.setMinimum(0)
        self.battery_progress.setMaximum(100)
        self.battery_progress.setTextVisible(True)
        self.battery_progress.setFormat('电量 %p%')
        self.battery_progress.setStyleSheet("""
            QProgressBar {
                border: 2px solid #ddd;
                border-radius: 8px;
                text-align: center;
                height: 35px;
                background-color: #f0f0f0;
                font-size: 12pt;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                          stop:0 #4CAF50, stop:1 #81C784);
                border-radius: 6px;
            }
        """)
        progress_layout.addWidget(self.battery_progress)
        layout.addWidget(progress_container)
        
        # 电池详细信息
        grid = QGridLayout()
        grid.setSpacing(8)
        
        self.battery_voltage_label = self.create_value_label('0.00 V')
        self.battery_current_label = self.create_value_label('0.00 A')
        self.battery_temp_label = self.create_value_label('0 °C')
        self.battery_time_label = self.create_value_label('0 min')
        self.battery_status_label = self.create_value_label('未知')
        self.battery_cycles_label = self.create_value_label('0')
        self.battery_capacity_label = self.create_value_label('0 mAh')
        
        # 使用简洁标签
        grid.addWidget(QLabel('电压:'), 0, 0)
        grid.addWidget(self.battery_voltage_label, 0, 1)
        grid.addWidget(QLabel('电流:'), 1, 0)
        grid.addWidget(self.battery_current_label, 1, 1)
        grid.addWidget(QLabel('温度:'), 2, 0)
        grid.addWidget(self.battery_temp_label, 2, 1)
        grid.addWidget(QLabel('预计使用:'), 3, 0)
        grid.addWidget(self.battery_time_label, 3, 1)
        grid.addWidget(QLabel('充电状态:'), 4, 0)
        grid.addWidget(self.battery_status_label, 4, 1)
        grid.addWidget(QLabel('循环次数:'), 5, 0)
        grid.addWidget(self.battery_cycles_label, 5, 1)
        grid.addWidget(QLabel('标称容量:'), 6, 0)
        grid.addWidget(self.battery_capacity_label, 6, 1)
        
        layout.addLayout(grid)
        group.setLayout(layout)
        return group
    
    def create_statistics_group(self):
        """创建统计信息组"""
        group = QGroupBox('■ 统计信息')
        grid = QGridLayout()
        
        self.total_distance_label = self.create_value_label('0 m')
        self.total_boot_time_label = self.create_value_label('0 s')
        self.total_boot_count_label = self.create_value_label('0')
        
        grid.addWidget(QLabel('总行驶里程:'), 0, 0)
        grid.addWidget(self.total_distance_label, 0, 1)
        grid.addWidget(QLabel('总开机时间:'), 1, 0)
        grid.addWidget(self.total_boot_time_label, 1, 1)
        grid.addWidget(QLabel('总开机次数:'), 2, 0)
        grid.addWidget(self.total_boot_count_label, 2, 1)
        
        group.setLayout(grid)
        return group
    
    def create_mission_group(self):
        """创建Mission信息组"""
        group = QGroupBox('◆ Mission 状态')
        grid = QGridLayout()
        
        self.mission_id_label = self.create_value_label('0')
        self.mission_status_label = self.create_value_label('未知')
        self.mission_result_label = self.create_value_label('未知')
        self.mission_error_label = self.create_value_label('0x00000000')
        
        grid.addWidget(QLabel('Mission ID:'), 0, 0)
        grid.addWidget(self.mission_id_label, 0, 1)
        grid.addWidget(QLabel('运行状态:'), 1, 0)
        grid.addWidget(self.mission_status_label, 1, 1)
        grid.addWidget(QLabel('执行结果:'), 2, 0)
        grid.addWidget(self.mission_result_label, 2, 1)
        grid.addWidget(QLabel('错误码:'), 3, 0)
        grid.addWidget(self.mission_error_label, 3, 1)
        
        group.setLayout(grid)
        return group
    
    def create_move_task_group(self):
        """创建移动任务信息组"""
        group = QGroupBox('► 移动任务状态')
        grid = QGridLayout()
        
        self.move_task_no_label = self.create_value_label('0')
        self.move_task_status_label = self.create_value_label('未知')
        self.move_task_result_label = self.create_value_label('未知')
        self.move_task_start_label = self.create_value_label('0')
        self.move_task_dest_label = self.create_value_label('0')
        self.move_task_path_label = self.create_value_label('0')
        
        grid.addWidget(QLabel('任务编号:'), 0, 0)
        grid.addWidget(self.move_task_no_label, 0, 1)
        grid.addWidget(QLabel('任务状态:'), 1, 0)
        grid.addWidget(self.move_task_status_label, 1, 1)
        grid.addWidget(QLabel('任务结果:'), 2, 0)
        grid.addWidget(self.move_task_result_label, 2, 1)
        grid.addWidget(QLabel('起始站点:'), 3, 0)
        grid.addWidget(self.move_task_start_label, 3, 1)
        grid.addWidget(QLabel('目标站点:'), 4, 0)
        grid.addWidget(self.move_task_dest_label, 4, 1)
        grid.addWidget(QLabel('路径编号:'), 5, 0)
        grid.addWidget(self.move_task_path_label, 5, 1)
        
        group.setLayout(grid)
        return group
    
    def create_status_flags_group(self):
        """创建机器人状态标志组"""
        group = QGroupBox('▼ 机器人状态')
        grid = QGridLayout()
        
        self.emergency_led = self.create_led_indicator()
        self.brake_led = self.create_led_indicator()
        self.charging_led = self.create_led_indicator()
        self.auto_mode_led = self.create_led_indicator()
        self.loaded_led = self.create_led_indicator()
        self.wifi_led = self.create_led_indicator()
        
        grid.addWidget(QLabel('急停解除:'), 0, 0)
        grid.addWidget(self.emergency_led, 0, 1)
        grid.addWidget(QLabel('抱闸释放:'), 1, 0)
        grid.addWidget(self.brake_led, 1, 1)
        grid.addWidget(QLabel('正在充电:'), 2, 0)
        grid.addWidget(self.charging_led, 2, 1)
        grid.addWidget(QLabel('自动模式:'), 3, 0)
        grid.addWidget(self.auto_mode_led, 3, 1)
        grid.addWidget(QLabel('已载荷:'), 4, 0)
        grid.addWidget(self.loaded_led, 4, 1)
        grid.addWidget(QLabel('WiFi连接:'), 5, 0)
        grid.addWidget(self.wifi_led, 5, 1)
        
        group.setLayout(grid)
        return group
    
    def create_obstacle_flags_group(self):
        """创建避障传感器状态组"""
        group = QGroupBox('◇ 避障传感器触发')
        grid = QGridLayout()
        
        self.obs_main_radar_led = self.create_led_indicator()
        self.obs_aux_radar_led = self.create_led_indicator()
        self.obs_camera1_led = self.create_led_indicator()
        self.obs_camera2_led = self.create_led_indicator()
        self.obs_radar1_led = self.create_led_indicator()
        self.obs_radar2_led = self.create_led_indicator()
        
        grid.addWidget(QLabel('主雷达:'), 0, 0)
        grid.addWidget(self.obs_main_radar_led, 0, 1)
        grid.addWidget(QLabel('辅雷达:'), 1, 0)
        grid.addWidget(self.obs_aux_radar_led, 1, 1)
        grid.addWidget(QLabel('深度相机1:'), 2, 0)
        grid.addWidget(self.obs_camera1_led, 2, 1)
        grid.addWidget(QLabel('深度相机2:'), 3, 0)
        grid.addWidget(self.obs_camera2_led, 3, 1)
        grid.addWidget(QLabel('避障雷达1:'), 4, 0)
        grid.addWidget(self.obs_radar1_led, 4, 1)
        grid.addWidget(QLabel('避障雷达2:'), 5, 0)
        grid.addWidget(self.obs_radar2_led, 5, 1)
        
        group.setLayout(grid)
        return group
    def create_manual_control_group(self):
        group = QGroupBox('● 手动控制')
        group.setObjectName('manualControl')
        grid = QGridLayout()
        toggle_row = QHBoxLayout()
        toggle_label = QLabel('启用手动控制')
        toggle_label.setStyleSheet('font-weight: bold; color: #2c5aa0;')
        self.manual_toggle = ToggleSwitch()
        self.manual_toggle.stateChanged.connect(self.on_manual_toggled)
        toggle_row.addWidget(toggle_label)
        toggle_row.addStretch()
        toggle_row.addWidget(self.manual_toggle)
        self.manual_hint = QLabel('W/A/S/D 前后左右，方向键左右为旋转')
        self.manual_hint.setStyleSheet('font-weight: bold;')
        self.manual_state_label = self.create_value_label('未启用')
        grid.addLayout(toggle_row, 0, 0, 1, 2)
        grid.addWidget(QLabel('当前状态:'), 1, 0)
        grid.addWidget(self.manual_state_label, 1, 1)
        grid.addWidget(self.manual_hint, 2, 0, 1, 2)
        group.setLayout(grid)
        return group
    def create_control_services_group(self):
        group = QGroupBox('● 控制服务')
        group.setObjectName('controlServices')
        grid = QGridLayout()
        grid.setSpacing(12)
        grid.setContentsMargins(8, 8, 8, 8)
        btn_pause_move = QPushButton('暂停移动')
        btn_resume_move = QPushButton('恢复移动')
        btn_stop_move = QPushButton('停止移动')
        btn_stop_loc = QPushButton('停止定位')
        btn_emg_stop = QPushButton('紧急停止')
        btn_release_emg = QPushButton('解除急停')
        btn_start_chg = QPushButton('开始充电')
        btn_stop_chg = QPushButton('停止充电')
        btn_enter_low = QPushButton('进入低功耗')
        btn_exit_low = QPushButton('退出低功耗')
        btn_sys_reset = QPushButton('系统复位')
        btn_pause_mis = QPushButton('任务暂停')
        btn_resume_mis = QPushButton('任务恢复')
        btn_cancel_mis = QPushButton('任务取消')
        btn_pause_move.clicked.connect(lambda: self.call_control(self.ros_node.cli_pause_move))
        btn_resume_move.clicked.connect(lambda: self.call_control(self.ros_node.cli_resume_move))
        btn_stop_move.clicked.connect(lambda: self.call_control(self.ros_node.cli_stop_move))
        btn_stop_loc.clicked.connect(lambda: self.call_control(self.ros_node.cli_stop_localization))
        btn_emg_stop.clicked.connect(lambda: self.call_control(self.ros_node.cli_emergency_stop))
        btn_release_emg.clicked.connect(lambda: self.call_control(self.ros_node.cli_release_emergency_stop))
        btn_start_chg.clicked.connect(lambda: self.call_control(self.ros_node.cli_start_charging))
        btn_stop_chg.clicked.connect(lambda: self.call_control(self.ros_node.cli_stop_charging))
        btn_enter_low.clicked.connect(lambda: self.call_control(self.ros_node.cli_enter_low_power))
        btn_exit_low.clicked.connect(lambda: self.call_control(self.ros_node.cli_exit_low_power))
        btn_sys_reset.clicked.connect(lambda: self.call_control(self.ros_node.cli_system_reset))
        btn_pause_mis.clicked.connect(lambda: self.call_control(self.ros_node.cli_pause_mission))
        btn_resume_mis.clicked.connect(lambda: self.call_control(self.ros_node.cli_resume_mission))
        btn_cancel_mis.clicked.connect(lambda: self.call_control(self.ros_node.cli_cancel_mission))
        widgets = [
            btn_pause_move, btn_resume_move, btn_stop_move, btn_stop_loc,
            btn_emg_stop, btn_release_emg, btn_start_chg, btn_stop_chg,
            btn_enter_low, btn_exit_low, btn_sys_reset,
            btn_pause_mis, btn_resume_mis, btn_cancel_mis,
        ]
        r = 0
        c = 0
        for w in widgets:
            w.setCursor(Qt.PointingHandCursor)
            w.setMinimumHeight(42)
            grid.addWidget(w, r, c)
            c += 1
            if c >= 3:
                c = 0
                r += 1
        group.setLayout(grid)
        return group
    
    def create_value_label(self, text):
        """创建值显示标签"""
        label = QLabel(text)
        label.setStyleSheet("""
            background-color: #f8f9fa; 
            padding: 8px 12px; 
            border: 1px solid #dee2e6; 
            border-radius: 5px;
            font-size: 10pt;
            color: #495057;
        """)
        label.setMinimumHeight(30)
        return label
    
    def create_led_indicator(self):
        """创建LED状态指示器"""
        label = QLabel('●')
        label.setAlignment(Qt.AlignCenter)
        font = QFont()
        font.setPointSize(24)
        font.setBold(True)
        label.setFont(font)
        label.setStyleSheet("""
            color: #cccccc; 
            background-color: #f5f5f5; 
            border: 2px solid #e0e0e0;
            border-radius: 22px;
        """)
        label.setFixedSize(44, 44)
        return label
    
    def set_led_status(self, led, active):
        """设置LED指示器状态"""
        if active:
            led.setStyleSheet("""
                color: #4CAF50; 
                background: qradialgradient(cx:0.5, cy:0.5, radius:0.8,
                                           fx:0.5, fy:0.5,
                                           stop:0 #81C784, stop:1 #4CAF50);
                border: 2px solid #2E7D32;
                border-radius: 22px;
            """)
        else:
            led.setStyleSheet("""
                color: #bdbdbd; 
                background-color: #f5f5f5; 
                border: 2px solid #e0e0e0;
                border-radius: 22px;
            """)
    def on_manual_toggled(self, state):
        enabled = state == Qt.Checked
        self.manual_enabled = enabled
        self.manual_state_label.setText('已启用' if enabled else '未启用')
        if enabled:
            self.call_control(self.ros_node.cli_start_manual)
            self.setFocus()
        else:
            self.cmd.forward = False
            self.cmd.backward = False
            self.cmd.rotate_left = False
            self.cmd.rotate_right = False
            self.ros_node.publish_manual(self.cmd)
            self.call_control(self.ros_node.cli_stop_manual)
    def call_control(self, client):
        req = client.srv_type.Request()
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.5)
        self.ros_node.call_service(client, req)
    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress:
            if self.manual_enabled and not event.isAutoRepeat():
                k = event.key()
                if k == Qt.Key_W or k == Qt.Key_Up:
                    self.cmd.forward = True
                    self.cmd.backward = False
                elif k == Qt.Key_S or k == Qt.Key_Down:
                    self.cmd.backward = True
                    self.cmd.forward = False
                elif k == Qt.Key_A or k == Qt.Key_Left:
                    self.cmd.rotate_left = True
                    self.cmd.rotate_right = False
                elif k == Qt.Key_D or k == Qt.Key_Right:
                    self.cmd.rotate_right = True
                    self.cmd.rotate_left = False
        elif event.type() == QEvent.KeyRelease:
            if self.manual_enabled and not event.isAutoRepeat():
                k = event.key()
                if k == Qt.Key_W or k == Qt.Key_Up:
                    self.cmd.forward = False
                elif k == Qt.Key_S or k == Qt.Key_Down:
                    self.cmd.backward = False
                elif k == Qt.Key_A or k == Qt.Key_Left:
                    self.cmd.rotate_left = False
                elif k == Qt.Key_D or k == Qt.Key_Right:
                    self.cmd.rotate_right = False
        return super().eventFilter(obj, event)
    def publish_manual_cmd(self):
        if self.manual_enabled:
            self.ros_node.publish_manual(self.cmd)
    
    def update_status(self, msg):
        """更新所有状态显示"""
        # 连接状态
        self.connection_label.setText('● 已连接')
        self.connection_label.setStyleSheet("""
            background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                      stop:0 #4CAF50, stop:1 #81C784);
            color: white; 
            padding: 8px 15px; 
            border-radius: 5px;
            font-weight: bold;
        """)
        
        # 系统状态
        self.sys_status_label.setText(self.get_system_status_text(msg.system_status))
        self.location_status_label.setText(self.get_location_status_text(msg.location_status))
        self.operation_status_label.setText(self.get_operation_status_text(msg.operation_status))
        self.scheduling_mode_label.setText(self.get_scheduling_mode_text(msg.scheduling_mode))
        self.motion_status_label.setText(self.get_motion_status_text(msg.motion_status))
        
        # 系统信息
        self.station_label.setText(str(msg.current_station_id))
        self.error_code_label.setText(f'0x{msg.last_error_code:08X}')
        self.volume_label.setText(str(msg.current_system_volume))
        ip_str = f'{msg.ip_addresses[0]}.{msg.ip_addresses[1]}.{msg.ip_addresses[2]}.{msg.ip_addresses[3]}'
        self.ip_label.setText(ip_str)
        self.map_label.setText(msg.current_map_name)
        
        # 位姿信息
        import math
        self.pos_x_label.setText(f'{msg.pose.pose.pose.position.x:.2f} m')
        self.pos_y_label.setText(f'{msg.pose.pose.pose.position.y:.2f} m')
        # 从四元数计算yaw
        q = msg.pose.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.yaw_label.setText(f'{yaw:.2f} rad ({math.degrees(yaw):.1f}°)')
        self.confidence_label.setText(f'{msg.pose_confidence * 100:.1f}%')
        
        # 速度信息
        self.vel_x_label.setText(f'{msg.twist.linear.x:.2f} m/s')
        self.vel_y_label.setText(f'{msg.twist.linear.y:.2f} m/s')
        self.omega_label.setText(f'{msg.twist.angular.z:.2f} rad/s')
        
        # 电池信息
        self.battery_progress.setValue(int(msg.battery_remaining_percentage))
        if msg.battery_remaining_percentage < 20:
            self.battery_progress.setStyleSheet("""
                QProgressBar {
                    border: 2px solid #ddd; border-radius: 8px; text-align: center; 
                    height: 35px; background-color: #f0f0f0; font-size: 12pt; font-weight: bold;
                }
                QProgressBar::chunk {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                              stop:0 #f44336, stop:1 #e57373);
                    border-radius: 6px;
                }
            """)
        elif msg.battery_remaining_percentage < 50:
            self.battery_progress.setStyleSheet("""
                QProgressBar {
                    border: 2px solid #ddd; border-radius: 8px; text-align: center; 
                    height: 35px; background-color: #f0f0f0; font-size: 12pt; font-weight: bold;
                }
                QProgressBar::chunk {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                              stop:0 #ff9800, stop:1 #ffb74d);
                    border-radius: 6px;
                }
            """)
        else:
            self.battery_progress.setStyleSheet("""
                QProgressBar {
                    border: 2px solid #ddd; border-radius: 8px; text-align: center; 
                    height: 35px; background-color: #f0f0f0; font-size: 12pt; font-weight: bold;
                }
                QProgressBar::chunk {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                                              stop:0 #4CAF50, stop:1 #81C784);
                    border-radius: 6px;
                }
            """)
        
        self.battery_voltage_label.setText(f'{msg.battery_voltage:.2f} V')
        self.battery_current_label.setText(f'{msg.battery_current:.2f} A')
        self.battery_temp_label.setText(f'{msg.battery_temperature} °C')
        self.battery_time_label.setText(f'{msg.battery_estimated_using_time} min')
        self.battery_status_label.setText(self.get_battery_status_text(msg.battery_status))
        self.battery_cycles_label.setText(str(msg.battery_cycle_count))
        self.battery_capacity_label.setText(f'{msg.battery_nominal_capacity} mAh')
        
        # 统计信息
        self.total_distance_label.setText(f'{msg.total_motion_distance} m')
        hours = msg.total_boot_time // 3600
        minutes = (msg.total_boot_time % 3600) // 60
        self.total_boot_time_label.setText(f'{hours}h {minutes}m')
        self.total_boot_count_label.setText(str(msg.total_boot_count))
        
        # 任务信息
        self.mission_id_label.setText(str(msg.current_mission_id))
        self.mission_status_label.setText(self.get_mission_status_text(msg.mission_status))
        self.mission_result_label.setText(self.get_mission_result_text(msg.mission_result))
        self.mission_error_label.setText(f'0x{msg.mission_error_code:08X}')
        
        self.move_task_no_label.setText(str(msg.current_move_task_no))
        self.move_task_status_label.setText(self.get_move_task_status_text(msg.move_task_status))
        self.move_task_result_label.setText(self.get_move_task_result_text(msg.move_task_result))
        self.move_task_start_label.setText(str(msg.current_move_task_start_station))
        self.move_task_dest_label.setText(str(msg.current_move_task_destination_station))
        self.move_task_path_label.setText(str(msg.current_move_task_path_no))
        
        # 状态标志
        self.set_led_status(self.emergency_led, not msg.is_emergency_stopped)
        self.set_led_status(self.brake_led, msg.is_brake_released)
        self.set_led_status(self.charging_led, msg.is_charging)
        self.set_led_status(self.auto_mode_led, msg.is_auto_mode)
        self.set_led_status(self.loaded_led, msg.is_loaded)
        self.set_led_status(self.wifi_led, msg.has_wifi)
        
        # 避障传感器
        self.set_led_status(self.obs_main_radar_led, msg.obstacle_avoidance_triggered_main_radar)
        self.set_led_status(self.obs_aux_radar_led, msg.obstacle_avoidance_triggered_aux_radar)
        self.set_led_status(self.obs_camera1_led, msg.obstacle_avoidance_triggered_depth_camera1)
        self.set_led_status(self.obs_camera2_led, msg.obstacle_avoidance_triggered_depth_camera2)
        self.set_led_status(self.obs_radar1_led, msg.obstacle_avoidance_triggered_obstacle_radar1)
        self.set_led_status(self.obs_radar2_led, msg.obstacle_avoidance_triggered_obstacle_radar2)
    
    # 状态文本转换函数
    def get_system_status_text(self, status):
        status_map = {
            0x01: '系统正在初始化', 0x02: '系统空闲', 0x03: '系统出错',
            0x04: '正在启动定位', 0x05: '导航正在初始化', 0x06: '导航正在寻路',
            0x07: '正在等待到达目标位置', 0x08: '检测到障碍，减速',
            0x09: '导航正在重新寻路', 0x0A: '遇到障碍暂停运动',
            0x0B: '无法抵达目标位置', 0x0E: '正在初始化执行固定路径',
            0x0F: '正在等待固定路径执行结束', 0x10: '执行固定路径检测到障碍，减速前进',
            0x11: '执行固定路径遇到障碍暂停运动', 0x12: '无法检测到目标站点',
            0x13: '执行固定路径时用户暂停了', 0x15: '导航过程中出错',
            0x16: '硬件错误'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')
    
    def get_location_status_text(self, status):
        status_map = {
            0x01: '定位未启动', 0x02: '定位正在初始化',
            0x03: '定位成功', 0x04: '正在重定位', 0x05: '定位错误'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')
    
    def get_operation_status_text(self, status):
        status_map = {0x00: '状态不可用', 0x01: '自动控制模式', 0x02: '手动控制模式'}
        return status_map.get(status, f'未知 (0x{status:02X})')
    
    def get_scheduling_mode_text(self, mode):
        mode_map = {0x01: '手动模式', 0x02: '自动模式', 0x03: '维保模式'}
        return mode_map.get(mode, f'未知 (0x{mode:02X})')
    
    def get_motion_status_text(self, status):
        status_map = {
            0x00: '未知状态', 0x01: '静止', 0x02: '前进', 0x03: '后退',
            0x04: '左转', 0x05: '右转', 0x06: '原地左转',
            0x07: '原地右转', 0x08: '左移', 0x09: '右移'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')
    
    def get_battery_status_text(self, status):
        status_map = {0x00: '无效状态', 0x02: '正在充电', 0x03: '未充电'}
        return status_map.get(status, f'未知 (0x{status:02X})')
    
    def get_mission_status_text(self, status):
        status_map = {
            0x00: '无效状态', 0x02: '在队列中', 0x03: '正在执行',
            0x04: '暂停执行', 0x05: '执行结束', 0x06: '正在取消'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')
    
    def get_mission_result_text(self, result):
        result_map = {
            0x00: '无效状态', 0x01: '任务执行成功',
            0x02: '任务取消', 0x03: '任务执行出错'
        }
        return result_map.get(result, f'未知 (0x{result:02X})')
    
    def get_move_task_status_text(self, status):
        status_map = {
            0x00: '无效状态', 0x02: '等待开始执行', 0x03: '正在执行',
            0x04: '暂停执行', 0x05: '执行结束', 0x06: '正在取消',
            0x08: '交通管制'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')
    
    def get_move_task_result_text(self, result):
        result_map = {
            0x00: '无效状态', 0x01: '任务执行成功',
            0x02: '任务取消', 0x03: '任务执行出错'
        }
        return result_map.get(result, f'未知 (0x{result:02X})')
    
    def closeEvent(self, event):
        """关闭窗口时清理资源"""
        self.ros_timer.stop()
        self.ros_node.destroy_node()
        rclpy.shutdown()
        event.accept()


def main(args=None):
    # 设置UTF-8编码
    import locale
    import os
    
    # 强制设置环境变量
    os.environ['LANG'] = 'zh_CN.UTF-8'
    os.environ['LC_ALL'] = 'zh_CN.UTF-8'
    
    try:
        locale.setlocale(locale.LC_ALL, 'zh_CN.UTF-8')
    except:
        try:
            locale.setlocale(locale.LC_ALL, 'C.UTF-8')
        except:
            pass
    
    app = QApplication(sys.argv)
    app.setStyle('Fusion')  # 使用Fusion样式以获得更好的跨平台外观
    
    # 设置字体 - 使用更常见的中文字体
    font = QFont()
    # 在 Linux 中常见的中文字体
    font.setFamily("Noto Sans CJK SC, WenQuanYi Micro Hei, Droid Sans Fallback, Arial Unicode MS, Sans Serif")
    font.setPointSize(10)
    app.setFont(font)
    
    window = RobotMonitorGUI()
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
