# Orbbec Gemini 335Le 相机使用指南

## 设备信息

| 项目 | 信息 |
|------|------|
| 相机型号 | Orbbec Gemini 335Le |
| 连接方式 | 以太网 (Ethernet) |
| PID | 0x080e |
| 固件版本 | 1.6.00 |
| IP 地址 | 192.168.1.10 |
| MAC 地址 | 54:14:FD:23:9E:7A |
| 子网掩码 | 255.255.255.0 |
| 网关 | 192.168.1.1 |

> **注意**: "Le" 后缀表示 Ethernet（以太网）版本，区别于 USB 版本的 Gemini 335。

---

## OrbbecSDK_ROS2 版本要求

### ⚠️ 重要：版本兼容性问题

在 **arm64** 平台（如 Jetson）上，OrbbecSDK_ROS2 存在版本兼容性问题：

| 版本 | 状态 | 问题 |
|------|------|------|
| v2.5.5 (tag) | ✅ 推荐 | 正常工作 |
| v2.5.4 | ❌ 不可用 | "Component 8 not found" 错误 |
| v2.4.7 | ⚠️ 部分可用 | 网络设备发现有问题 |
| v2.4.4 | ⚠️ 部分可用 | 网络设备发现有问题 |
| v1.5.8 | ✅ 可用 | 旧版本，功能较少 |

### 推荐版本

```bash
# 使用 v2.5.5 tag 版本
git clone https://gitee.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2.5.5
```

---

## 安装步骤

### 1. 安装依赖

```bash
sudo apt install libgflags-dev nlohmann-json3-dev \
    ros-humble-image-transport ros-humble-image-transport-plugins \
    ros-humble-compressed-image-transport ros-humble-image-publisher \
    ros-humble-camera-info-manager ros-humble-diagnostic-updater \
    ros-humble-diagnostic-msgs ros-humble-statistics-msgs \
    ros-humble-backward-ros libdw-dev
```

### 2. 克隆仓库并切换版本

```bash
cd ~/your_ws/src
git clone https://gitee.com/orbbec/OrbbecSDK_ROS2.git
cd OrbbecSDK_ROS2
git checkout v2.5.5
```

### 3. 安装 udev 规则

```bash
cd OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 4. 编译

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orbbec_camera_msgs orbbec_description orbbec_camera \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 网络配置

### 主机网络设置

确保主机的以太网接口与相机在同一网段：

```bash
# 查看网络接口
ip addr show

# 示例配置 (eno1 接口)
# IP: 192.168.1.191/24
# 相机 IP: 192.168.1.10
```

### 验证网络连通性

```bash
# 检查 ARP 表（相机可能不响应 ping）
arp -a | grep 192.168.1.10

# 应该看到类似输出:
# ? (192.168.1.10) 位于 54:14:fd:23:9e:7a [ether] 在 eno1
```

---

## 使用方法

### 检测设备

```bash
source ~/your_ws/install/setup.bash
ros2 run orbbec_camera list_devices_node
```

预期输出：
```
[INFO] - Name: Orbbec Gemini 335Le, PID: 0x080e, SN/ID: CPE895300029, Connection: Ethernet
[INFO] serial: CPE895300029
[INFO] ip address: 192.168.1.10
[INFO] usb connect type: Ethernet
[INFO] MAC address: 54:14:FD:23:9E:7A
```

### 启动相机

```bash
source ~/your_ws/install/setup.bash
ros2 launch orbbec_camera gemini_330_series.launch.py
```

### 指定 IP 启动（可选）

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py net_device_ip:=192.168.1.10
```

---

## ROS2 话题

启动成功后，以下话题可用：

| 话题 | 类型 | 描述 |
|------|------|------|
| `/camera/color/image_raw` | sensor_msgs/Image | 彩色图像 |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | 彩色相机内参 |
| `/camera/depth/image_raw` | sensor_msgs/Image | 深度图像 |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | 深度相机内参 |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | 点云数据 |
| `/camera/device_status` | orbbec_camera_msgs/DeviceStatus | 设备状态 |

### 查看话题

```bash
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/depth/image_raw
```

---

## 常用参数配置

### 分辨率和帧率

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
    color_width:=1280 color_height:=800 color_fps:=30 \
    depth_width:=1280 depth_height:=800 depth_fps:=30
```

### 启用点云

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
    enable_point_cloud:=true \
    enable_colored_point_cloud:=true
```

### 深度对齐到彩色

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
    depth_registration:=true \
    align_mode:=SW
```

### 启用 IR 流

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
    enable_left_ir:=true \
    enable_right_ir:=true
```

### PTP 时间同步（仅 Gemini 335Le 支持）

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py \
    enable_ptp_config:=true
```

---

## 故障排除

### 问题 1: "Component 8 not found" 错误

**原因**: SDK 版本不兼容

**解决方案**:
```bash
cd ~/your_ws/src/OrbbecSDK_ROS2
git checkout v2.5.5
cd ~/your_ws
colcon build --packages-select orbbec_camera_msgs orbbec_description orbbec_camera
```

### 问题 2: 设备未找到

**检查步骤**:
1. 确认网络连接正常
   ```bash
   ip addr show | grep 192.168.1
   ```
2. 检查 ARP 表
   ```bash
   arp -a | grep 192.168.1.10
   ```
3. 确认 SDK 配置启用网络枚举
   ```bash
   grep EnumerateNetDevice ~/your_ws/src/OrbbecSDK_ROS2/orbbec_camera/config/OrbbecSDKConfig_v2.0.xml
   # 应该看到: <EnumerateNetDevice>true</EnumerateNetDevice>
   ```

### 问题 3: libOrbbecSDK.so 符号链接问题

**解决方案**:
```bash
cd ~/your_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/
ls -la libOrbbecSDK.so*

# 如果符号链接损坏，重新创建:
rm -f libOrbbecSDK.so libOrbbecSDK.so.2
ln -s libOrbbecSDK.so.2.5.5 libOrbbecSDK.so.2
ln -s libOrbbecSDK.so.2 libOrbbecSDK.so
```

### 问题 4: device_online: false

**可能原因**: 
- 网络配置错误
- 相机 IP 不在同一网段
- 旧版本 SDK 的网络发现功能有问题

**解决方案**: 使用 v2.5.5 版本

---

## 支持的传感器

Gemini 335Le 支持以下传感器：

| 传感器类型 | 描述 |
|-----------|------|
| OB_SENSOR_COLOR | 彩色相机 |
| OB_SENSOR_DEPTH | 深度相机 |
| OB_SENSOR_IR_LEFT | 左红外相机 |
| OB_SENSOR_IR_RIGHT | 右红外相机 |
| OB_SENSOR_ACCEL | 加速度计 |
| OB_SENSOR_GYRO | 陀螺仪 |

> **注意**: Gemini 335Le **不支持** OB_SENSOR_RAW_PHASE (Component 8)，这是导致某些版本出现 "Component 8 not found" 错误的原因。

---

## 参考资料

- [OrbbecSDK_ROS2 GitHub](https://github.com/orbbec/OrbbecSDK_ROS2)
- [OrbbecSDK_ROS2 Gitee 镜像](https://gitee.com/orbbec/OrbbecSDK_ROS2)
- [Orbbec 官网](https://www.orbbec.com/)
- [GitHub Issue #149](https://github.com/orbbec/OrbbecSDK_ROS2/issues/149) - Component 8 问题讨论

---

*文档更新日期: 2025年12月3日*
