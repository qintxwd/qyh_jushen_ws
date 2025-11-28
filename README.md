# QYH Jushen 机器人工作空间

## 系统依赖安装

### 1. ROS2 基础依赖
```bash
# ROS2 消息和工具
sudo apt-get install -y ros-humble-rosidl-typesupport-c \
                        ros-humble-rosidl-default-generators \
                        ros-humble-rosidl-generator-py \
                        ros-humble-ament-cmake \
                        ros-humble-ament-cmake-python

# Python工具
sudo apt-get install -y python3-pip \
                        python3-ament-package \
                        python3-colcon-common-extensions \
                        python3-rosdep \
                        python3-vcstool
```

### 2. 硬件接口依赖
```bash
# 串口通信（夹爪、底盘）
sudo apt-get install -y python3-serial \
                        ros-humble-serial-driver

# 网络通信
sudo apt-get install -y libboost-all-dev
```

### 3. MoveIt2 和运动规划
```bash
# MoveIt2 核心
sudo apt install -y ros-humble-moveit \
                    ros-humble-moveit-servo \
                    ros-humble-moveit-visual-tools \
                    ros-humble-moveit-ros-planning-interface \
                    ros-humble-moveit-planners-ompl

# 控制和平滑
sudo apt install -y ros-humble-control-toolbox \
                    ros-humble-controller-manager \
                    ros-humble-ros2-control \
                    ros-humble-ros2-controllers
```

### 4. 机器人描述和可视化
```bash
# URDF和TF
sudo apt install -y ros-humble-xacro \
                    ros-humble-robot-state-publisher \
                    ros-humble-joint-state-publisher \
                    ros-humble-joint-state-publisher-gui

# RViz和可视化
sudo apt install -y ros-humble-rviz2 \
                    ros-humble-rviz-visual-tools \
                    ros-humble-interactive-markers
```

### 5. 传感器和感知
```bash
# 相机支持
sudo apt install -y ros-humble-cv-bridge \
                    ros-humble-image-transport \
                    ros-humble-image-transport-plugins \
                    ros-humble-vision-opencv

# OpenCV
sudo apt install -y libopencv-dev \
                    python3-opencv
```

### 6. VR遥操作依赖（如需要）
```bash
# TF2变换
sudo apt install -y ros-humble-tf2 \
                    ros-humble-tf2-ros \
                    ros-humble-tf2-geometry-msgs \
                    ros-humble-tf2-eigen

# 几何和数学库
sudo apt install -y libeigen3-dev \
                    ros-humble-geometry-msgs \
                    ros-humble-sensor-msgs
```

### 7. 开发和调试工具
```bash
# 编译工具
sudo apt install -y build-essential \
                    cmake \
                    git

# 调试工具
sudo apt install -y ros-humble-rqt \
                    ros-humble-rqt-common-plugins \
                    ros-humble-plotjuggler-ros

# 性能分析
sudo apt install -y ros-humble-ros2trace \
                    ros-humble-tracetools
```

### 8. 网络和通信
```bash
# WebSocket（如用于Web界面）
sudo apt install -y libwebsocketpp-dev \
                    nlohmann-json3-dev

# ADB工具（如用于VR设备调试）
sudo apt install -y android-tools-adb \
                    android-tools-fastboot
```

### 9. Python依赖
```bash
# 【推荐】配置国内pip镜像源（阿里云）加速下载
mkdir -p ~/.pip
cat > ~/.pip/pip.conf << 'EOF'
[global]
index-url = https://mirrors.aliyun.com/pypi/simple/
trusted-host = mirrors.aliyun.com
EOF

# ROS2消息生成必需（必须使用3.3.4版本，与ROS2 Humble兼容）
pip3 install 'empy==3.3.4'

# 基础Python库
pip3 install numpy scipy matplotlib

# ROS2 Python工具
pip3 install transforms3d

# 如需要数据采集
pip3 install h5py pandas
```

**重要**: 
- `empy` 是 ROS2 编译自定义消息包（如 `qyh_lift_msgs`、`qyh_gripper_msgs` 等）时必需的依赖。如果缺少此包，编译时会报错 `ModuleNotFoundError: No module named 'em'`。
- 使用阿里云镜像源可以大幅提升国内下载速度。

### 10. 一键安装脚本
```bash
# 创建安装脚本
cat > install_dependencies.sh << 'EOF'
#!/bin/bash
set -e

echo "=== 安装ROS2 Humble依赖 ==="

# 1. ROS2基础
sudo apt-get update
sudo apt-get install -y \
    ros-humble-rosidl-typesupport-c \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-generator-py \
    ros-humble-ament-cmake \
    ros-humble-ament-cmake-python \
    python3-pip \
    python3-ament-package \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# 2. 硬件接口
sudo apt-get install -y \
    python3-serial \
    ros-humble-serial-driver \
    libboost-all-dev

# 3. MoveIt2
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-visual-tools \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-planners-ompl \
    ros-humble-control-toolbox \
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers

# 4. 机器人描述
sudo apt install -y \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    ros-humble-rviz-visual-tools \
    ros-humble-interactive-markers

# 5. 传感器
sudo apt install -y \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-image-transport-plugins \
    ros-humble-vision-opencv \
    libopencv-dev \
    python3-opencv

# 6. VR和TF
sudo apt install -y \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf2-eigen \
    libeigen3-dev \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs

# 7. 开发工具
sudo apt install -y \
    build-essential \
    cmake \
    git \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-plotjuggler-ros

# 8. 网络通信
sudo apt install -y \
    libwebsocketpp-dev \
    nlohmann-json3-dev

# 9. Python依赖
# 配置国内镜像源（推荐）
mkdir -p ~/.pip
cat > ~/.pip/pip.conf << 'PIPEOF'
[global]
index-url = https://mirrors.aliyun.com/pypi/simple/
trusted-host = mirrors.aliyun.com
PIPEOF

# ROS2消息生成必需
pip3 install 'empy==3.3.4'

# 其他Python库
pip3 install numpy scipy matplotlib transforms3d h5py pandas

echo "=== 依赖安装完成 ==="
echo "请运行: source /opt/ros/humble/setup.bash"
EOF

chmod +x install_dependencies.sh
```

### 11. 运行安装
```bash
# 执行安装脚本
./install_dependencies.sh

# 或者分步安装（如果遇到问题）
# 按照上面1-9的顺序逐步执行
```

---

## 硬件配置

### 网络配置
- **底盘 AP模式**: 192.168.10.10
- **底盘 网口**: 192.168.71.50
- **机械臂 网口**: 192.168.2.200

### 串口设备
- **夹爪测试**: ttyUSB0

---

## 编译工作空间

```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 常见问题

### 依赖缺失错误
如果编译时提示找不到某个包：
```bash
# 搜索包名
apt search ros-humble-<package-name>

# 安装特定包
sudo apt install ros-humble-<package-name>
```

### rosdep更新
```bash
# 初始化rosdep
sudo rosdep init
rosdep update

# 自动安装依赖
cd ~/qyh_jushen_ws/qyh_jushen_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 清理构建
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
rm -rf build install log
colcon build --symlink-install
```