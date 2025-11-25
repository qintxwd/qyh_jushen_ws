# 🚀 JAKA 伺服测试 - 快速入门

## 📋 一分钟快速开始

```bash
# 1. 进入目录
cd ~/qyh_jushen_ws/资料/双机械臂/jakaAPI_K1_2.3.3_DUAL_x64/example/c++

# 2. 添加执行权限
chmod +x build_and_run.sh

# 3. 运行一键脚本
./build_and_run.sh
```

就这么简单！脚本会自动完成：
✅ 检查SDK库文件
✅ 配置CMake
✅ 编译所有测试程序
✅ 检查机器人连接
✅ 提供安全提示
✅ 运行选择的测试

## 🎯 测试程序说明

### 1️⃣ 关节伺服测试（推荐首选）

**特点：**
- 正弦波关节运动
- 频率：125Hz（8ms周期）
- 幅度：安全范围内的小幅度运动

**适合场景：**
- 验证伺服控制功能
- 测试实时性能
- 对比ROS2程序效果

### 2️⃣ 力控伺服测试

**特点：**
- 笛卡尔空间运动
- 启用力传感器
- 带力控反馈

**注意：**
需要机器人配备力传感器

### 3️⃣ 多臂MoveJ测试

**特点：**
- 双臂同步运动
- 关节空间轨迹
- 点到点运动

### 4️⃣ 多臂MoveL测试

**特点：**
- 双臂同步运动
- 笛卡尔空间直线
- 点到点运动

## 🔧 手动编译（如果脚本有问题）

```bash
# 创建build目录
mkdir -p build
cd build

# 配置
cmake ..

# 编译
make -j$(nproc)

# 运行
sudo ./edgservo_test
```

## ⚠️ 常见问题

### Q: 找不到库文件？

```bash
# 检查库文件是否存在
ls -la ../../Linux/c\&c++/x86_64-linux-gnu/shared/libjakaAPI_2_3_3.so

# 如果存在但运行报错，设置环境变量
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$(pwd)/../../Linux/c\&c++/x86_64-linux-gnu/shared
```

### Q: 连接不到机器人？

```bash
# 测试网络
ping 192.168.2.200

# 检查IP是否正确
# 如果IP不同，修改.cpp文件中的IP地址
```

### Q: 机器人不动？

检查清单：
- [ ] 机器人已上电
- [ ] 使能成功（无错误）
- [ ] 急停按钮已松开
- [ ] 机器人无错误状态

### Q: 需要修改运动参数？

编辑 `30.edgservo.cpp`：

```cpp
// 降低速度（第85行附近）
double kk = 0.5;  // 改小这个值

// 降低幅度（第90-92行附近）
jpos_cmd.jVal[0] = sin(kk*t) * deg2rad(10);  // 改小这些值
```

## 📝 与ROS2对比测试

如果SDK测试正常但ROS2不行：

```bash
# 1. 先运行SDK测试，观察运动效果
sudo ./edgservo_test

# 2. 记录运动参数
# - 运动是否流畅
# - 速度和幅度
# - 实时性如何

# 3. 运行ROS2程序对比
# 在另一个终端：
ros2 launch qyh_jaka_control_gui jaka_gui.launch.py

# 4. 对比差异
# - 运动是否相似
# - 是否有抖动
# - 延迟情况
```

## 📚 更多信息

详细文档请查看：
- **README.md** - 完整使用说明
- **CMakeLists.txt** - 编译配置
- **FEATURES.md** - SDK功能特性

## 💡 提示

运行测试前务必：
1. ✅ 确认周围无障碍物
2. ✅ 准备好急停按钮
3. ✅ 保持安全距离
4. ✅ 全程监控

---

**有问题？** 查看 README.md 的故障排查章节
