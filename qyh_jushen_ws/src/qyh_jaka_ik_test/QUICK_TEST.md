# 多客户端连接测试指南

## 🎯 测试目的

验证JAKA SDK是否允许多个客户端同时连接到控制器。

## 📝 测试方法

提供两种测试方法：

### 方法1：Python脚本测试（推荐 - 不需要编译）

**步骤：**

1. **启动主控制节点：**
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash
ros2 launch qyh_jaka_control jaka_control.launch.py
```

等待看到连接成功消息：
```
✓ Connected to robot at 192.168.2.200
```

2. **运行测试脚本（新终端）：**
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws/src/qyh_jaka_ik_test/scripts
chmod +x test_multi_client.py
python3 test_multi_client.py
```

### 方法2：C++节点测试（需要Jetson环境）

qyh_jaka_ik_test包需要在Jetson上编译（SDK是ARM64版本）。

**在Jetson上：**
```bash
cd ~/qyh_jushen_ws/qyh_jushen_ws
colcon build --packages-select qyh_jaka_ik_test
source install/setup.bash

# 终端1
ros2 launch qyh_jaka_control jaka_control.launch.py

# 终端2
ros2 launch qyh_jaka_ik_test ik_test.launch.py
```

## ✅ 成功标志

### Python脚本测试：
```
✅ 测试1: 成功
✅ 测试2: 成功
🎉 SDK支持多客户端连接！
```

### C++节点测试：
```
[jaka_control_node] ✓ Connected to robot at 192.168.2.200
[ik_test_node] ✅ 成功连接到控制器！
[ik_test_node] 📊 IK统计: 总计=100, 成功=100, 失败=0
```

## ❌ 失败情况

如果看到：
- 第二个连接被拒绝
- 第一个连接断开
- 错误：SDK不支持多客户端

**建议：** 在单个节点内集成IK功能

## 🔧 故障排查

### 问题1：第一次连接就失败
```bash
# 测试网络
ping 192.168.2.200

# 检查端口
nc -zv 192.168.2.200 10000
```

### 问题2：C++包编译失败
- 原因：SDK是ARM64格式，需要在Jetson上编译
- 解决：使用Python脚本测试即可

## 📊 测试记录

**日期：** _______________

**环境：**
- [ ] WSL (仅Python测试)
- [ ] Jetson (完整测试)

**测试结果：**
- [ ] 多客户端连接成功
- [ ] 多客户端连接失败

**结论：**
```
[ ] SDK支持多客户端 → 创建 qyh_dual_arm_ik_solver 独立包
[ ] SDK不支持多连接 → 在 qyh_jaka_control 内集成IK
```
