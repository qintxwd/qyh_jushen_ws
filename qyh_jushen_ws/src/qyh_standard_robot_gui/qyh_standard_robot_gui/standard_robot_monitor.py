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
from qyh_standard_robot_msgs.msg import NavigationStatus
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
    GoSetSpeedType,
    GoNavigateToCoordinate,
    GoExecuteActionTask,
    GoSetManualControl,
    GoSetObstacleStrategy,
    GoSetCurrentSite,
    GoSetSpeakerVolume,
    GoSetCurrentMap,
    GoForceLocalize,
    GoNavigateToPoseWithTask,
    GoNavigateToSiteWithTask,
)


class RobotStatusSignals(QObject):
    """信号类，用于线程安全的UI更新"""
    status_updated = pyqtSignal(object)
    nav_status_updated = pyqtSignal(object)


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
        self.latest_nav_status = None
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
        
        # Go series service clients
        self.cli_go_set_speed = self.create_client(GoSetSpeedType, 'go_set_speed_level')
        self.cli_go_nav_coord = self.create_client(GoNavigateToCoordinate, 'go_navigate_to_coordinate')
        self.cli_go_nav_site = self.create_client(GoExecuteActionTask, 'go_navigate_to_site')
        self.cli_go_manual = self.create_client(GoSetManualControl, 'go_set_manual_control')
        self.cli_go_obstacle = self.create_client(GoSetObstacleStrategy, 'go_set_obstacle_strategy')
        self.cli_go_current_site = self.create_client(GoSetCurrentSite, 'go_set_current_site')
        self.cli_go_volume = self.create_client(GoSetSpeakerVolume, 'go_set_speaker_volume')
        self.cli_go_map = self.create_client(GoSetCurrentMap, 'go_set_current_map')
        self.cli_go_force_loc = self.create_client(GoForceLocalize, 'go_force_localize')
        self.cli_go_nav_pose_task = self.create_client(GoNavigateToPoseWithTask, 'go_navigate_to_pose_with_task')
        self.cli_go_nav_site_task = self.create_client(GoNavigateToSiteWithTask, 'go_navigate_to_site_with_task')
        
        # 订阅机器人状态
        self.subscription = self.create_subscription(
            StandardRobotStatus,
            'standard_robot_status',
            self.status_callback,
            10)
        
        # 订阅导航状态
        self.nav_subscription = self.create_subscription(
            NavigationStatus,
            'navigation_status',
            self.nav_status_callback,
            10)
        
        self.get_logger().info('Standard Robot Monitor started')
    
    def status_callback(self, msg):
        self.latest_status = msg
        self.signals.status_updated.emit(msg)
    
    def nav_status_callback(self, msg):
        self.latest_nav_status = msg
        self.signals.nav_status_updated.emit(msg)
    
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
                color: black;
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
                color: black;
            }
            QGroupBox#controlServices QPushButton:hover {
                background-color: #1976d2;
            }
            QGroupBox#controlServices QPushButton:pressed {
                background-color: #0d47a1;
            }
            QComboBox {
                background-color: white;
                color: black;
                border: 2px solid #4a90e2;
                border-radius: 5px;
                padding: 5px 10px;
                font-size: 10pt;
                min-height: 25px;
            }
            QComboBox:hover {
                border: 2px solid #2c5aa0;
            }
            QComboBox::drop-down {
                border: none;
                width: 30px;
            }
            QComboBox::down-arrow {
                image: none;
                border-left: 5px solid transparent;
                border-right: 5px solid transparent;
                border-top: 8px solid #4a90e2;
                margin-right: 8px;
            }
            QComboBox QAbstractItemView {
                background-color: white;
                color: black;
                selection-background-color: #4a90e2;
                selection-color: white;
                border: 2px solid #4a90e2;
                outline: none;
                padding: 5px;
            }
            QComboBox QAbstractItemView::item {
                min-height: 30px;
                padding: 5px 10px;
                color: black;
            }
            QComboBox QAbstractItemView::item:hover {
                background-color: #e8f4f8;
                color: black;
            }
            QComboBox QAbstractItemView::item:selected {
                background-color: #4a90e2;
                color: white;
            }
            QLineEdit {
                background-color: white;
                color: black;
                border: 2px solid #4a90e2;
                border-radius: 5px;
                padding: 5px 10px;
                font-size: 10pt;
                min-height: 25px;
            }
            QLineEdit:focus {
                border: 2px solid #2c5aa0;
            }
        """)
        
        # 初始化ROS2
        self.signals = RobotStatusSignals()
        self.signals.status_updated.connect(self.update_status)
        self.signals.nav_status_updated.connect(self.update_nav_status)
        
        rclpy.init()
        self.ros_node = StandardRobotMonitor(self.signals)
        
        # 创建定时器来处理ROS2回调
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros_node)
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
    
    def spin_ros_node(self):
        if rclpy.ok():
            try:
                rclpy.spin_once(self.ros_node, timeout_sec=0)
            except Exception:
                pass

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
        
        # 第五行：手动控制和控制服务
        row5 = QHBoxLayout()
        row5.addWidget(self.create_manual_control_group(), 2)
        row5.addWidget(self.create_control_services_group(), 3)
        parent_layout.addLayout(row5)
        
        # 第六行：导航控制
        row6 = QHBoxLayout()
        row6.addWidget(self.create_navigation_control_group())
        parent_layout.addLayout(row6)
        
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
    
    def create_navigation_control_group(self):
        """创建导航控制组"""
        from PyQt5.QtWidgets import QLineEdit, QTabWidget
        
        group = QGroupBox('◆ 定位与导航控制 (Go服务)')
        group.setObjectName('navigationControl')
        main_layout = QHBoxLayout()
        
        # 左侧：使用Tab分类功能
        input_widget = QWidget()
        input_layout = QVBoxLayout(input_widget)
        
        # 创建标签页
        tab_widget = QTabWidget()
        
        # === 定位功能标签页 ===
        localization_tab = QWidget()
        loc_layout = QVBoxLayout(localization_tab)
        
        # 位姿定位
        pose_loc_group = QGroupBox('通过位姿定位 (40001-40006)')
        pose_loc_layout = QGridLayout()
        
        self.loc_pose_x_input = QLineEdit('0.0')
        self.loc_pose_y_input = QLineEdit('0.0')
        self.loc_pose_yaw_input = QLineEdit('0.0')
        
        pose_loc_layout.addWidget(QLabel('X (米):'), 0, 0)
        pose_loc_layout.addWidget(self.loc_pose_x_input, 0, 1)
        pose_loc_layout.addWidget(QLabel('Y (米):'), 1, 0)
        pose_loc_layout.addWidget(self.loc_pose_y_input, 1, 1)
        pose_loc_layout.addWidget(QLabel('Yaw (弧度):'), 2, 0)
        pose_loc_layout.addWidget(self.loc_pose_yaw_input, 2, 1)
        
        btn_loc_pose = QPushButton('通过位姿定位')
        btn_loc_pose.clicked.connect(self.on_localize_by_pose)
        pose_loc_layout.addWidget(btn_loc_pose, 3, 0, 1, 2)
        
        pose_loc_group.setLayout(pose_loc_layout)
        loc_layout.addWidget(pose_loc_group)
        
        # 站点定位
        site_loc_group = QGroupBox('通过站点定位 (40007)')
        site_loc_layout = QGridLayout()
        
        self.loc_site_input = QLineEdit('0')
        
        site_loc_layout.addWidget(QLabel('站点ID:'), 0, 0)
        site_loc_layout.addWidget(self.loc_site_input, 0, 1)
        
        btn_loc_site = QPushButton('通过站点定位')
        btn_loc_site.clicked.connect(self.on_localize_by_site)
        site_loc_layout.addWidget(btn_loc_site, 1, 0, 1, 2)
        
        site_loc_group.setLayout(site_loc_layout)
        loc_layout.addWidget(site_loc_group)
        
        # 强制定位
        force_loc_group = QGroupBox('强制定位 (40049,40051,40053)')
        force_loc_layout = QGridLayout()
        
        self.force_loc_x_input = QLineEdit('0.0')
        self.force_loc_y_input = QLineEdit('0.0')
        self.force_loc_yaw_input = QLineEdit('0.0')
        
        force_loc_layout.addWidget(QLabel('X (米):'), 0, 0)
        force_loc_layout.addWidget(self.force_loc_x_input, 0, 1)
        force_loc_layout.addWidget(QLabel('Y (米):'), 1, 0)
        force_loc_layout.addWidget(self.force_loc_y_input, 1, 1)
        force_loc_layout.addWidget(QLabel('Yaw (弧度):'), 2, 0)
        force_loc_layout.addWidget(self.force_loc_yaw_input, 2, 1)
        
        btn_force_loc = QPushButton('强制定位')
        btn_force_loc.clicked.connect(self.on_force_localize)
        force_loc_layout.addWidget(btn_force_loc, 3, 0, 1, 2)
        
        force_loc_group.setLayout(force_loc_layout)
        loc_layout.addWidget(force_loc_group)
        
        loc_layout.addStretch()
        tab_widget.addTab(localization_tab, '定位功能')
        
        # === 导航功能标签页 ===
        navigation_tab = QWidget()
        nav_layout = QVBoxLayout(navigation_tab)
        
        # 自主导航到位姿
        nav_pose_group = QGroupBox('导航到位姿 (40008-40013)')
        nav_pose_layout = QGridLayout()
        
        self.nav_pose_x_input = QLineEdit('0.0')
        self.nav_pose_y_input = QLineEdit('0.0')
        self.nav_pose_yaw_input = QLineEdit('0.0')
        
        nav_pose_layout.addWidget(QLabel('X (米):'), 0, 0)
        nav_pose_layout.addWidget(self.nav_pose_x_input, 0, 1)
        nav_pose_layout.addWidget(QLabel('Y (米):'), 1, 0)
        nav_pose_layout.addWidget(self.nav_pose_y_input, 1, 1)
        nav_pose_layout.addWidget(QLabel('Yaw (弧度):'), 2, 0)
        nav_pose_layout.addWidget(self.nav_pose_yaw_input, 2, 1)
        
        btn_nav_pose = QPushButton('导航到位姿')
        btn_nav_pose.clicked.connect(self.on_navigate_to_pose)
        nav_pose_layout.addWidget(btn_nav_pose, 3, 0, 1, 2)
        
        nav_pose_group.setLayout(nav_pose_layout)
        nav_layout.addWidget(nav_pose_group)
        
        # 自主导航到站点
        nav_site_group = QGroupBox('导航到站点 (40015)')
        nav_site_layout = QGridLayout()
        
        self.nav_site_input = QLineEdit('0')
        
        nav_site_layout.addWidget(QLabel('站点ID:'), 0, 0)
        nav_site_layout.addWidget(self.nav_site_input, 0, 1)
        
        btn_nav_site = QPushButton('导航到站点')
        btn_nav_site.clicked.connect(self.on_navigate_to_site)
        nav_site_layout.addWidget(btn_nav_site, 1, 0, 1, 2)
        
        nav_site_group.setLayout(nav_site_layout)
        nav_layout.addWidget(nav_site_group)
        
        nav_layout.addStretch()
        tab_widget.addTab(navigation_tab, '导航功能')
        
        input_layout.addWidget(tab_widget)
        
        # 参数设置
        param_group = QGroupBox('参数设置')
        param_layout = QGridLayout()
        
        self.speed_level_input = QLineEdit('50')
        self.obstacle_strategy_input = QLineEdit('0')
        self.current_site_input = QLineEdit('0')
        self.speaker_volume_input = QLineEdit('50')
        self.current_map_input = QLineEdit('12')
        
        param_layout.addWidget(QLabel('速度级别 [1-100]:'), 0, 0)
        param_layout.addWidget(self.speed_level_input, 0, 1)
        btn_set_speed = QPushButton('设置')
        btn_set_speed.clicked.connect(self.on_set_speed_level)
        param_layout.addWidget(btn_set_speed, 0, 2)
        
        param_layout.addWidget(QLabel('避障策略:'), 1, 0)
        param_layout.addWidget(self.obstacle_strategy_input, 1, 1)
        btn_set_obstacle = QPushButton('设置')
        btn_set_obstacle.clicked.connect(self.on_set_obstacle_strategy)
        param_layout.addWidget(btn_set_obstacle, 1, 2)
        
        param_layout.addWidget(QLabel('当前站点:'), 2, 0)
        param_layout.addWidget(self.current_site_input, 2, 1)
        btn_set_site = QPushButton('设置')
        btn_set_site.clicked.connect(self.on_set_current_site)
        param_layout.addWidget(btn_set_site, 2, 2)
        
        param_layout.addWidget(QLabel('音量 [1-100]:'), 3, 0)
        param_layout.addWidget(self.speaker_volume_input, 3, 1)
        btn_set_volume = QPushButton('设置')
        btn_set_volume.clicked.connect(self.on_set_speaker_volume)
        param_layout.addWidget(btn_set_volume, 3, 2)
        
        param_layout.addWidget(QLabel('地图名 (2字符):'), 4, 0)
        param_layout.addWidget(self.current_map_input, 4, 1)
        btn_set_map = QPushButton('设置')
        btn_set_map.clicked.connect(self.on_set_current_map)
        param_layout.addWidget(btn_set_map, 4, 2)
        
        param_group.setLayout(param_layout)
        input_layout.addWidget(param_group)
        
        input_layout.addStretch()
        main_layout.addWidget(input_widget, 2)
        
        # 右侧：导航状态显示
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        
        nav_status_group = QGroupBox('导航状态')
        nav_status_layout = QGridLayout()
        
        self.nav_comm_pose_label = self.create_value_label('0, 0, 0')
        self.nav_auto_pose_label = self.create_value_label('0, 0, 0')
        self.nav_speed_level_label = self.create_value_label('0')
        self.nav_obstacle_label = self.create_value_label('0')
        self.nav_site_label = self.create_value_label('0')
        self.nav_volume_label = self.create_value_label('0')
        
        nav_status_layout.addWidget(QLabel('定位位姿(40001-40006):'), 0, 0)
        nav_status_layout.addWidget(self.nav_comm_pose_label, 0, 1)
        nav_status_layout.addWidget(QLabel('导航位姿(40008-40013):'), 1, 0)
        nav_status_layout.addWidget(self.nav_auto_pose_label, 1, 1)
        nav_status_layout.addWidget(QLabel('速度级别:'), 2, 0)
        nav_status_layout.addWidget(self.nav_speed_level_label, 2, 1)
        nav_status_layout.addWidget(QLabel('避障策略:'), 3, 0)
        nav_status_layout.addWidget(self.nav_obstacle_label, 3, 1)
        nav_status_layout.addWidget(QLabel('当前站点:'), 4, 0)
        nav_status_layout.addWidget(self.nav_site_label, 4, 1)
        nav_status_layout.addWidget(QLabel('音量:'), 5, 0)
        nav_status_layout.addWidget(self.nav_volume_label, 5, 1)
        
        nav_status_group.setLayout(nav_status_layout)
        status_layout.addWidget(nav_status_group)
        
        status_layout.addStretch()
        main_layout.addWidget(status_widget, 3)
        
        group.setLayout(main_layout)
        return group
    
    def on_localize_by_pose(self):
        """通过位姿定位 (40001-40006) - 仅定位，不移动"""
        try:
            req = GoNavigateToCoordinate.Request()
            req.x = float(self.loc_pose_x_input.text())
            req.y = float(self.loc_pose_y_input.text())
            req.yaw = float(self.loc_pose_yaw_input.text())
            req.is_localization = True  # 使用40001-40006, 仅定位
            self.call_service_with_feedback(
                self.ros_node.cli_go_nav_coord, req, '通过位姿定位'
            )
        except ValueError:
            self.ros_node.get_logger().error('输入值无效')
    
    def on_localize_by_site(self):
        """通过站点定位 (40007) - 仅定位，不移动"""
        try:
            req = GoExecuteActionTask.Request()
            req.site_id = int(self.loc_site_input.text())
            req.is_localization = True  # 使用40007, 仅定位
            self.call_service_with_feedback(
                self.ros_node.cli_go_nav_site, req, '通过站点定位'
            )
        except ValueError:
            self.ros_node.get_logger().error('输入值无效')
    
    def on_navigate_to_pose(self):
        """导航到位姿 (40008-40013) - 移动到目标位置"""
        try:
            req = GoNavigateToCoordinate.Request()
            req.x = float(self.nav_pose_x_input.text())
            req.y = float(self.nav_pose_y_input.text())
            req.yaw = float(self.nav_pose_yaw_input.text())
            req.is_localization = False  # 使用40008-40013, 导航移动
            self.call_service_with_feedback(
                self.ros_node.cli_go_nav_coord, req, '导航到位姿'
            )
        except ValueError:
            self.ros_node.get_logger().error('输入值无效')
    
    def on_navigate_to_coordinate(self):
        """导航到坐标（已废弃，使用on_localize_by_pose或on_navigate_to_pose）"""
        pass
    
    def on_force_localize(self):
        """强制定位 (40049,40051,40053)"""
        try:
            req = GoForceLocalize.Request()
            req.x = float(self.force_loc_x_input.text())
            req.y = float(self.force_loc_y_input.text())
            req.yaw = float(self.force_loc_yaw_input.text())
            self.call_service_with_feedback(self.ros_node.cli_go_force_loc, req, '强制定位')
        except ValueError:
            self.ros_node.get_logger().error('输入值无效')
    
    def on_navigate_to_site(self):
        """导航到站点 (40015) - 移动到目标站点"""
        try:
            req = GoExecuteActionTask.Request()
            req.site_id = int(self.nav_site_input.text())
            req.is_localization = False  # 使用40015, 导航移动
            self.call_service_with_feedback(
                self.ros_node.cli_go_nav_site, req, '导航到站点'
            )
        except ValueError:
            self.ros_node.get_logger().error('输入值无效')
    
    def on_set_speed_level(self):
        """设置速度级别"""
        try:
            req = GoSetSpeedType.Request()
            req.speed_level = int(self.speed_level_input.text())
            self.call_service_with_feedback(self.ros_node.cli_go_set_speed, req, '设置速度级别')
        except ValueError:
            self.ros_node.get_logger().error('输入值无效')
    
    def on_set_obstacle_strategy(self):
        """设置避障策略"""
        try:
            req = GoSetObstacleStrategy.Request()
            req.strategy = int(self.obstacle_strategy_input.text())
            self.call_service_with_feedback(self.ros_node.cli_go_obstacle, req, '设置避障策略')
        except ValueError:
            self.ros_node.get_logger().error('输入值无效')
    
    def on_set_current_site(self):
        """设置当前站点"""
        try:
            req = GoSetCurrentSite.Request()
            req.site_id = int(self.current_site_input.text())
            self.call_service_with_feedback(self.ros_node.cli_go_current_site, req, '设置当前站点')
        except ValueError:
            self.ros_node.get_logger().error('输入值无效')
    
    def on_set_speaker_volume(self):
        """设置音量"""
        try:
            req = GoSetSpeakerVolume.Request()
            req.volume = int(self.speaker_volume_input.text())
            self.call_service_with_feedback(self.ros_node.cli_go_volume, req, '设置音量')
        except ValueError:
            self.ros_node.get_logger().error('输入值无效')
    
    def on_set_current_map(self):
        """设置当前地图"""
        req = GoSetCurrentMap.Request()
        req.map_name = self.current_map_input.text()
        self.call_service_with_feedback(self.ros_node.cli_go_map, req, '设置地图')
    
    def call_service_with_feedback(self, client, request, action_name):
        """调用服务并显示反馈"""
        if not client.service_is_ready():
            client.wait_for_service(timeout_sec=0.5)
        
        future = self.ros_node.call_service(client, request)
        if future:
            self.ros_node.get_logger().info(f'{action_name} 服务已调用')
    
    def update_nav_status(self, msg):
        """更新导航状态显示"""
        self.nav_comm_pose_label.setText(
            f'{msg.communication_pose.x:.2f}, {msg.communication_pose.y:.2f}, '
            f'{msg.communication_pose.yaw:.2f}'
        )
        self.nav_auto_pose_label.setText(
            f'{msg.autonomous_nav_pose.x:.2f}, {msg.autonomous_nav_pose.y:.2f}, '
            f'{msg.autonomous_nav_pose.yaw:.2f}'
        )
        self.nav_speed_level_label.setText(str(msg.speed_level))
        self.nav_obstacle_label.setText(str(msg.obstacle_strategy))
        self.nav_site_label.setText(str(msg.current_site))
        self.nav_volume_label.setText(str(msg.speaker_volume))
    
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
    
    # 允许 Ctrl+C 退出程序
    import signal
    signal.signal(signal.SIGINT, lambda *args: app.quit())
    
    window = RobotMonitorGUI()
    window.show()
    
    # 运行应用
    exit_code = app.exec_()
    
    # 清理资源
    if rclpy.ok():
        rclpy.shutdown()
        
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
