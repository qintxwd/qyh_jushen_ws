# JAKA 双臂机器人伺服测试程序

JAKA SDK 版本: **2.3.3 DUAL** (K1系列双臂机器人专用)

本目录包含JAKA双臂机器人的伺服运动测试示例程序。

## 📁 文件说明

### 伺服测试程序
- **30.edgservo.cpp** - 关节空间伺服运动测试（正弦波轨迹，125Hz）
- **31.edgservo_fct.cpp** - 笛卡尔空间力控伺服测试
- **32.controlloop.cpp** - 控制环测试（如果存在）

### 点到点运动程序
- **23.multi_movj.cpp** - 双臂关节空间运动
- **24.multi_movl.cpp** - 双臂笛卡尔空间直线运动

### 其他测试
- **40.error_and_clear.cpp** - 错误处理
- **41.collision.cpp** - 碰撞检测
- **42.enable.cpp** - 使能控制
- **43.tool_offset.cpp** - 工具坐标偏移
- **44.base_offset.cpp** - 基座坐标偏移

## 🔧 SDK结构

```
jakaAPI_K1_2.3.3_DUAL_x64/
├── Linux/
│   └── c&c++/
│       ├── inc_of_c++/           # C++头文件
│       │   ├── JAKAZuRobot.h    # 主API接口
│       │   ├── jkerr.h          # 错误码定义
│       │   └── jktypes.h        # 类型定义
│       └── x86_64-linux-gnu/
│           └── shared/
│               └── libjakaAPI_2_3_3.so  # x86_64库
├── example/
│   └── c++/                      # 示例程序目录（当前目录）
└── FEATURES.md                   # SDK功能特性说明
```

## 🚀 快速开始

### 1. 编译程序

#### 方法1：使用一键脚本（推荐）

```bash
# 在WSL中进入示例目录
cd ~/qyh_jushen_ws/资料/双机械臂/jakaAPI_K1_2.3.3_DUAL_x64/example/c++

# 给脚本添加执行权限
chmod +x build_and_run.sh

# 运行脚本
./build_and_run.sh
```

#### 方法2：手动编译

```bash
# 创建build目录
mkdir -p build
cd build

# 运行CMake配置
cmake ..

# 编译（使用多核加速）
make -j$(nproc)

# 查看生成的可执行文件
ls -lh *_test
```

### 2. 运行测试程序

```bash
# 在build目录中

# 1. 关节伺服测试（推荐先运行这个）
sudo ./edgservo_test

# 2. 力控伺服测试
sudo ./edgservo_fct_test

# 3. 多臂MoveJ测试
sudo ./multi_movj_test

# 4. 多臂MoveL测试
sudo ./multi_movl_test
```

**注意：** 需要sudo权限以获取实时调度优先级（SCHED_FIFO）

## 📊 30.edgservo 测试说明

### 测试流程

1. ✅ 连接机器人（IP: 192.168.2.200）
2. ✅ 关闭所有伺服模式
3. ✅ 急停复位
4. ✅ 上电（power_on）
5. ✅ 使能（enable_robot）
6. ✅ 等待5秒
7. ✅ 移动到零位
8. ✅ 启动伺服模式（1ms周期）
9. 🔄 执行正弦波运动（无限循环）

### 运动参数

```cpp
// 控制周期
#define CONTROL_LOOP_MS 8    // 8ms = 125Hz

// 频率系数（控制运动速度）
double kk = 3;               // 值越大运动越快

// 运动幅度（度）
joint_0 = sin(kk*t) * 30;           // ±30度
joint_1 = -cos(kk*t) * 20 + 20;     // 0~40度
joint_3 = -cos(kk*t) * 10 + 10;     // 0~20度
```

### 修改为更安全的参数

如果觉得运动太快或幅度太大，修改示例代码：

```cpp
// 在30.edgservo.cpp中修改：

// 1. 降低频率（运动更慢）
double kk = 0.5;  // 改为0.5，速度降为原来的1/6

// 2. 降低幅度（运动范围更小）
jpos_cmd.jVal[0] = sin(kk*t) * deg2rad(10);      // 改为±10度
jpos_cmd.jVal[1] = -cos(kk*t) * deg2rad(8) + deg2rad(8);  // 改为0~16度
jpos_cmd.jVal[3] = -cos(kk*t) * deg2rad(5) + deg2rad(5);  // 改为0~10度
```

## 🔍 关键API函数

### 初始化和连接
```cpp
JAKAZuRobot robot;
robot.login_in("192.168.2.200");       // 连接控制器
robot.login_out();                      // 断开连接
```

### 基础控制
```cpp
robot.power_on();                       // 上电
robot.enable_robot();                   // 使能
robot.motion_abort();                   // 急停
```

### 伺服模式
```cpp
robot.servo_move_enable(1, 0);          // 启动左臂伺服模式
robot.servo_move_enable(1, 1);          // 启动右臂伺服模式
robot.servo_move_enable(0, -1);         // 关闭所有伺服模式
robot.servo_move_use_none_filter();     // 不使用滤波器
robot.servo_move_use_joint_LPF(125.0);  // 使用低通滤波（125Hz）
```

### 伺服运动控制
```cpp
// 关节空间伺服
robot.edg_recv(&next);                  // 接收状态
robot.edg_get_stat(0, &jpos, &cpos);   // 获取左臂状态
robot.edg_servo_j(0, &jpos_cmd, MoveMode::ABS);  // 发送关节指令
robot.edg_send();                       // 发送命令

// 笛卡尔空间伺服
robot.edg_servo_p(0, &cpos_cmd, MoveMode::ABS);  // 发送位姿指令
```

### 点到点运动
```cpp
robot.robot_run_multi_movj(LEFT, mode, true, jpos, v, a);   // 左臂MoveJ
robot.robot_run_multi_movl(RIGHT, mode, true, cpos, v, a);  // 右臂MoveL
robot.robot_run_multi_movj(DUAL, mode, true, jpos, v, a);   // 双臂同步MoveJ
```

## ⚙️ 系统要求

### 软件要求
- **OS**: Ubuntu 20.04 / 22.04
- **Kernel**: Linux 实时内核（推荐但非必须）
- **CMake**: >= 3.10
- **GCC**: >= 7.0（支持C++11）

### 硬件要求
- **CPU**: x86_64 架构（WSL/PC）或 aarch64（Jetson）
- **网络**: 能够访问 192.168.2.200
- **内存**: >= 2GB

### 权限要求
```bash
# 需要实时调度权限
sudo ./program_name

# 或者配置用户权限（永久生效）
sudo usermod -a -G realtime $USER
echo "@realtime - rtprio 99" | sudo tee -a /etc/security/limits.conf
```

## 🐛 故障排查

### 编译错误

**错误1：找不到头文件**
```
fatal error: JAKAZuRobot.h: No such file or directory
```
**解决：** 检查SDK路径是否正确
```bash
ls -la ../../Linux/c\&c++/inc_of_c++/JAKAZuRobot.h
```

**错误2：找不到库文件**
```
cannot find -ljakaAPI_2_3_3
```
**解决：** 检查库文件是否存在
```bash
ls -la ../../Linux/c\&c++/x86_64-linux-gnu/shared/libjakaAPI_2_3_3.so
```

### 运行错误

**错误1：连接被拒绝**
```
Connection refused to 192.168.2.200
```
**解决方案：**
```bash
# 1. 检查网络连接
ping 192.168.2.200

# 2. 检查防火墙
sudo ufw status

# 3. 检查机器人控制器是否开机
```

**错误2：权限不足**
```
Permission denied
```
**解决方案：**
```bash
# 使用sudo运行
sudo ./edgservo_test

# 或者配置实时权限（见上文"权限要求"）
```

**错误3：机器人不动**
```
程序运行但机器人无反应
```
**检查清单：**
- [ ] 机器人是否上电（LED灯亮起）
- [ ] 是否使能成功（无错误提示）
- [ ] 是否在伺服模式下（检查日志）
- [ ] 急停按钮是否松开
- [ ] 机器人是否有错误状态（清除错误）

### 库文件加载错误

**错误：运行时找不到so库**
```
error while loading shared libraries: libjakaAPI_2_3_3.so
```
**解决方案：**
```bash
# 方法1：设置LD_LIBRARY_PATH（临时）
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/qyh_jushen_ws/资料/双机械臂/jakaAPI_K1_2.3.3_DUAL_x64/Linux/c\&c++/x86_64-linux-gnu/shared

# 方法2：复制库文件到系统目录（永久）
sudo cp ../../Linux/c\&c++/x86_64-linux-gnu/shared/libjakaAPI_2_3_3.so /usr/local/lib/
sudo ldconfig

# 方法3：使用RPATH（CMakeLists.txt已配置）
# 编译时自动设置，直接运行即可
```

## 🆚 与ROS2程序对比

### SDK示例特点
✅ 实时性更好（直接调用SDK）
✅ 控制更精确（无ROS消息序列化开销）
✅ 延迟更低（~1ms vs ~8ms）
✅ 适合高频控制（可达1000Hz）

### ROS2程序特点
✅ 架构更清晰（话题/服务分离）
✅ 可视化更好（RViz，rqt）
✅ 集成更方便（与其他ROS包）
✅ 调试更容易（rostopic, rosservice）

### 如果SDK正常但ROS2不行

对比检查项：
1. **消息格式**：
   - SDK: `JointValue jpos[2]`（每个7个关节）
   - ROS2: `positions[14]`（14个关节数组）

2. **控制频率**：
   - SDK: 8ms周期（125Hz）
   - ROS2: 检查QTimer是否真正达到125Hz

3. **数据单位**：
   - SDK: 弧度（rad）
   - ROS2: 确认是弧度不是角度

4. **运动模式**：
   - SDK: `MoveMode::ABS`（绝对位置）
   - ROS2: `is_abs = True`

5. **初始化**：
   - SDK: 自动移动到零位
   - ROS2: 检查是否在安全位置

## 📝 修改IP地址

默认控制器IP是`192.168.2.200`，如需修改：

```cpp
// 在各个.cpp文件中找到并修改：
robot.login_in("192.168.2.200");  // 改为你的IP
```

或者使用环境变量：
```bash
export JAKA_IP="192.168.2.100"
# 然后修改代码读取环境变量
```

## 📚 参考资料

- **SDK文档**: `jakaAPI_K1_2.3.3_DUAL_x64/FEATURES.md`
- **API头文件**: `Linux/c&c++/inc_of_c++/JAKAZuRobot.h`
- **错误码**: `Linux/c&c++/inc_of_c++/jkerr.h`
- **类型定义**: `Linux/c&c++/inc_of_c++/jktypes.h`

## ⚠️ 安全提示

1. **首次运行前**：
   - ✅ 确保机器人周围无障碍物
   - ✅ 确保急停按钮功能正常
   - ✅ 建议先在仿真环境测试
   - ✅ 熟悉急停操作流程

2. **运行过程中**：
   - 👀 全程监控机器人运动
   - 👆 随时准备按急停按钮
   - 📏 保持安全距离（≥1米）
   - ⚡ 注意运动速度和幅度

3. **异常情况**：
   - 🛑 立即按急停
   - 📝 记录错误信息
   - 🔍 检查日志输出
   - 📞 联系技术支持

## 🎯 下一步

1. **成功运行SDK示例后**，对比ROS2程序：
   - 检查消息格式是否匹配
   - 检查控制频率是否达标
   - 检查数据转换是否正确

2. **优化ROS2程序**：
   - 参考SDK示例的实时性设置
   - 使用相同的滤波器配置
   - 确保发布频率稳定

3. **部署到Jetson**：
   - 编译ARM64版本
   - 测试实际硬件
   - 验证VR跟随功能

---

**最后编译时间**: 2025-11-25  
**SDK版本**: jakaAPI_K1_2.3.3_DUAL_x64  
**测试平台**: Ubuntu 22.04 + WSL2
