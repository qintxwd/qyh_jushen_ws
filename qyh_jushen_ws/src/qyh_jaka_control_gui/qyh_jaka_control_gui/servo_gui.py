#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGroupBox, QGridLayout, QCheckBox
from PyQt5.QtCore import QTimer, Qt, QEvent
from PyQt5.QtGui import QFont
from qyh_jaka_control_msgs.msg import JakaDualJointServo, JakaDualCartesianServo
from qyh_jaka_control_msgs.srv import StartServo, StopServo

class ServoPublisher(Node):
    def __init__(self):
        super().__init__('servo_publisher')
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE)
        self.joint_pub = self.create_publisher(JakaDualJointServo, '/jaka/servo/joint_cmd', qos)
        self.cart_pub = self.create_publisher(JakaDualCartesianServo, '/jaka/servo/cartesian_cmd', qos)
        self.cli_start = self.create_client(StartServo, '/jaka/servo/start')
        self.cli_stop = self.create_client(StopServo, '/jaka/servo/stop')
    def publish_joint(self, msg: JakaDualJointServo):
        self.joint_pub.publish(msg)
    def start(self):
        if not self.cli_start.service_is_ready():
            self.cli_start.wait_for_service(timeout_sec=0.5)
        req = StartServo.Request()
        self.cli_start.call_async(req)
    def stop(self):
        if not self.cli_stop.service_is_ready():
            self.cli_stop.wait_for_service(timeout_sec=0.5)
        req = StopServo.Request()
        self.cli_stop.call_async(req)

class JakaServoGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('JAKA Servo GUI')
        self.setGeometry(100, 100, 600, 400)
        self.publisher = ServoPublisher()
        self.manual_enabled = False
        self.is_abs = True
        self.cmd = JakaDualJointServo()
        self.cmd.positions = [0.0]*14
        self.cmd.is_abs = True
        self.cmd.velocity = 0.5
        self.cmd.acceleration = 1.0
        self.installEventFilter(self)
        self._init_ui()
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(50)  # 20Hz

    def _init_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        header = QLabel('◆ JAKA 伺服控制 ◆')
        f = QFont(); f.setPointSize(16); f.setBold(True)
        header.setFont(f)
        layout.addWidget(header)
        row = QHBoxLayout()
        row.addWidget(self._create_control_group(), 1)
        layout.addLayout(row)
        central.setFocusPolicy(Qt.StrongFocus)
        self.setFocusPolicy(Qt.StrongFocus)

    def _create_control_group(self):
        group = QGroupBox('● 控制')
        grid = QGridLayout()
        self.toggle = QCheckBox('启用伺服')
        self.toggle.stateChanged.connect(self._on_toggle)
        self.mode = QCheckBox('绝对模式(ABS)')
        self.mode.setChecked(True)
        self.mode.stateChanged.connect(self._on_mode)
        self.state_label = QLabel('未启用')
        grid.addWidget(self.toggle, 0, 0, 1, 2)
        grid.addWidget(self.mode, 1, 0, 1, 2)
        grid.addWidget(QLabel('状态:'), 2, 0)
        grid.addWidget(self.state_label, 2, 1)
        group.setLayout(grid)
        return group

    def _on_toggle(self, state):
        self.manual_enabled = (state == Qt.Checked)
        self.state_label.setText('已启用' if self.manual_enabled else '未启用')
        if self.manual_enabled:
            self.publisher.start()
        else:
            self.publisher.stop()

    def _on_mode(self, state):
        self.is_abs = (state == Qt.Checked)
        self.cmd.is_abs = self.is_abs

    def eventFilter(self, obj, event):
        if event.type() == QEvent.KeyPress and self.manual_enabled and not event.isAutoRepeat():
            k = event.key()
            # 简单演示：用方向键调整第一个关节角度
            if k == Qt.Key_Left:
                self.cmd.positions[0] -= 0.05
            elif k == Qt.Key_Right:
                self.cmd.positions[0] += 0.05
            elif k == Qt.Key_Up:
                self.cmd.positions[1] += 0.05
            elif k == Qt.Key_Down:
                self.cmd.positions[1] -= 0.05
        return super().eventFilter(obj, event)

    def _tick(self):
        if self.manual_enabled:
            self.publisher.publish_joint(self.cmd)

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = JakaServoGUI()
    gui.show()
    # ROS spin via timer
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(gui.publisher, timeout_sec=0))
    ros_timer.start(10)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()