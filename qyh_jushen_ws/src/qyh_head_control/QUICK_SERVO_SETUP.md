# 舵机ID设置快速指南

## ⚡ 快速开始

### 在Jetson上操作（必需！）

```bash
# 1. 只连接Tilt舵机（上下转动的那个）到控制板
# 2. 运行交互式工具
cd ~/qyh_jushen_ws/qyh_jushen_ws
source install/setup.bash
ros2 run qyh_head_control set_servo_id

# 3. 选择选项 2，确认设置为ID=2
# 4. 完成！
```

## 📝 为什么需要设置？

两个舵机出厂ID都是1，必须将Tilt舵机改为ID=2：
- **Pan舵机** (左右转动): ID = 1 ✅ 保持出厂设置
- **Tilt舵机** (上下转动): ID = 2 ⚠️ 需要修改

## 🔧 三种设置方法

### 方法一：交互式工具（最简单）

```bash
# 只连接Tilt舵机
ros2 run qyh_head_control set_servo_id
# 选择选项 2
```

### 方法二：Launch文件（自动化）

```bash
# 只连接Tilt舵机
ros2 launch qyh_head_control set_servo_id.launch.py old_id:=1 new_id:=2
```

### 方法三：原始SDK工具

```bash
cd ~/qyh_jushen_ws/资料/头部电机/06\ Jetson\ Nano版本/程序文件/案例2\ 总线舵机ID设置/
python3 set_serial_servo_status.py
```

## ✅ 验证设置

```bash
# 连接两个舵机，运行：
ros2 run qyh_head_control set_servo_id

# 应该看到：
#   ✓ 发现舵机 ID=1
#   ✓ 发现舵机 ID=2
```

## ⚠️ 重要提示

1. **必须单独连接**：每次只连接一个舵机
2. **在Jetson上操作**：WSL/Windows无法运行（缺少GPIO硬件）
3. **断开第一个再连第二个**：避免ID冲突

## 📖 详细文档

完整说明参见：[SERVO_ID_SETUP.md](SERVO_ID_SETUP.md)
