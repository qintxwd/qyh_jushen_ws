# JAKA Control 多架构编译指南

## 目录结构验证

重组完成后的 thirdparty 目录结构：

```
thirdparty/
├── arm64/
│   ├── include/
│   │   ├── JAKAZuRobot.h
│   │   ├── jkerr.h
│   │   └── jktypes.h
│   └── lib/
│       └── libjakaAPI_2_3_0_13.so
└── x64/
    ├── include/
    │   ├── JAKAZuRobot.h
    │   ├── jkerr.h
    │   └── jktypes.h
    └── lib/
        └── libjakaAPI_2_3_3.so
```

## 编译说明

### 在 ARM64 平台（Jetson/嵌入式）

```bash
cd ~/qyh-robot-system/qyh_jushen_ws
source /opt/ros/humble/setup.bash

# 清理旧的构建（可选）
rm -rf build/qyh_jaka_control install/qyh_jaka_control

# 编译
colcon build --packages-select qyh_jaka_control --symlink-install

# 预期输出：
# 🔧 Detected ARM64 architecture
# 📁 JAKA Include directory: .../thirdparty/arm64/include
# 📁 JAKA Library directory: .../thirdparty/arm64/lib
# ✅ Found JAKA API library: libjakaAPI_2_3_0_13.so
```

### 在 x64 平台（PC/服务器）

```bash
cd ~/qyh-robot-system/qyh_jushen_ws
source /opt/ros/humble/setup.bash

# 清理旧的构建（可选）
rm -rf build/qyh_jaka_control install/qyh_jaka_control

# 编译
colcon build --packages-select qyh_jaka_control --symlink-install

# 预期输出：
# 🔧 Detected x64 architecture
# 📁 JAKA Include directory: .../thirdparty/x64/include
# 📁 JAKA Library directory: .../thirdparty/x64/lib
# ✅ Found JAKA API library: libjakaAPI_2_3_3.so
```

## 运行时验证

### 检查链接的库

**ARM64:**
```bash
ldd install/qyh_jaka_control/lib/qyh_jaka_control/jaka_control_node | grep jaka
# 应该显示: libjakaAPI_2_3_0_13.so
```

**x64:**
```bash
ldd install/qyh_jaka_control/lib/qyh_jaka_control/jaka_control_node | grep jaka
# 应该显示: libjakaAPI_2_3_3.so
```

### 测试节点启动

```bash
source install/setup.bash
ros2 run qyh_jaka_control jaka_control_node --ros-args -p robot_ip:="192.168.2.200"

# 应该能正常连接机器人
```

## 故障排查

### 1. 编译时找不到库

**错误信息：**
```
FATAL_ERROR: JAKA API shared library not found in .../thirdparty/xxx/lib
```

**解决方案：**
- 检查对应架构的 lib 目录是否存在库文件
- 验证目录结构是否正确重组
- 查看 CMake 输出的架构检测信息

### 2. 架构检测错误

**如果检测到错误的架构：**

手动指定架构（编辑 CMakeLists.txt）：
```cmake
# 强制使用 arm64
set(JAKA_ARCH "arm64")

# 或强制使用 x64
set(JAKA_ARCH "x64")
```

### 3. 运行时找不到 .so 文件

**错误信息：**
```
error while loading shared libraries: libjakaAPI_xxx.so: cannot open shared object file
```

**解决方案：**

临时方案：
```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/qyh-robot-system/qyh_jushen_ws/install/qyh_jaka_control/lib/qyh_jaka_control/thirdparty
```

永久方案（已在 CMakeLists.txt 中配置 RPATH）：
```bash
# 重新编译即可
colcon build --packages-select qyh_jaka_control --symlink-install
```

## 交叉编译（高级）

### 在 x64 上为 ARM64 编译

需要安装交叉编译工具链：

```bash
# 安装 ARM64 工具链
sudo apt-get install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# 配置交叉编译
export CC=aarch64-linux-gnu-gcc
export CXX=aarch64-linux-gnu-g++

# 编译
colcon build --packages-select qyh_jaka_control \
  --cmake-args -DCMAKE_SYSTEM_PROCESSOR=aarch64
```

## SDK 版本差异

### ARM64 (2.3.0.13)
- Jetson 优化
- 实时性增强
- 支持 edg_get_stat 扭矩反馈

### x64 (2.3.3)
- 更新的功能
- 桌面调试支持
- 完整的开发工具支持

⚠️ **注意**：两个版本的 API 接口基本兼容，但部分高级功能可能有差异。

## 参考

- [JAKA SDK 官方文档](https://www.jaka.com)
- [CMake 架构检测文档](https://cmake.org/cmake/help/latest/variable/CMAKE_SYSTEM_PROCESSOR.html)
- [ROS2 多平台支持](https://docs.ros.org/en/humble/Installation.html)
