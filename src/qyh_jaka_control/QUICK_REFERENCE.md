# qyh_jaka_control 多架构支持速查

## ✅ 已完成的修改

1. **目录结构重组**
   - ✅ 移动 `thirdparty/include` → `thirdparty/arm64/include`
   - ✅ 移动 `thirdparty/lib` → `thirdparty/arm64/lib`
   - ✅ 保持 `thirdparty/x64/` 结构不变

2. **CMakeLists.txt 更新**
   - ✅ 自动架构检测 (ARM64/x64)
   - ✅ 动态库路径选择
   - ✅ 编译时信息输出
   - ✅ RPATH 自动配置

3. **文档更新**
   - ✅ README.md - SDK 版本与架构说明
   - ✅ MULTI_ARCH_BUILD.md - 详细编译指南
   - ✅ reorganize_thirdparty.sh/ps1 - 自动化脚本

## 📂 最终目录结构

```
qyh_jaka_control/
├── CMakeLists.txt              (已修改 - 支持多架构)
├── README.md                   (已更新 - 新增架构说明)
├── docs/
│   └── MULTI_ARCH_BUILD.md    (新增 - 编译指南)
├── scripts/
│   ├── reorganize_thirdparty.sh   (新增)
│   └── reorganize_thirdparty.ps1  (新增)
└── thirdparty/
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

## 🚀 快速开始

### 编译（任意平台）

```bash
cd ~/qyh-robot-system/qyh_jushen_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select qyh_jaka_control --symlink-install
```

CMake 会自动：
- 检测系统架构
- 选择对应的库路径
- 配置正确的头文件
- 输出编译信息

### 验证架构

编译时查看输出：
```
🔧 Detected ARM64 architecture  (或 x64 architecture)
📁 JAKA Include directory: .../thirdparty/arm64/include
📁 JAKA Library directory: .../thirdparty/arm64/lib
✅ Found JAKA API library: libjakaAPI_2_3_0_13.so
```

## 📋 检查清单

在提交代码前确认：

- [ ] 目录结构已按照规范重组
- [ ] ARM64 平台编译通过
- [ ] x64 平台编译通过（如可用）
- [ ] 节点能正常启动
- [ ] 能连接到机器人（如可用）
- [ ] 文档已更新

## 🔧 常用命令

```bash
# 清理重新编译
rm -rf build/qyh_jaka_control install/qyh_jaka_control
colcon build --packages-select qyh_jaka_control

# 检查链接的库
ldd install/qyh_jaka_control/lib/qyh_jaka_control/jaka_control_node | grep jaka

# 测试运行
source install/setup.bash
ros2 run qyh_jaka_control jaka_control_node --ros-args -p robot_ip:="192.168.2.200"
```

## 💡 提示

- 两个架构的头文件版本不同，但 API 基本兼容
- ARM64 SDK 针对 Jetson 优化，性能更好
- x64 SDK 版本更新，适合开发调试
- RPATH 已配置，无需手动设置 LD_LIBRARY_PATH

## 📞 遇到问题？

1. 查看 [MULTI_ARCH_BUILD.md](docs/MULTI_ARCH_BUILD.md) 故障排查章节
2. 检查 CMake 输出的架构检测信息
3. 验证对应架构的库文件是否存在
4. 确认 ROS2 环境已正确 source
