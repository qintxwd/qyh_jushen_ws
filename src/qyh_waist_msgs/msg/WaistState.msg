# 腰部电机状态消息
# 控制机器人身体前倾角度
# 位置范围: 163711(前倾45°) ~ 230715(竖直0°)
# Modbus TCP: 地址待定

std_msgs/Header header

# === 状态显示 ===
bool enabled              # 是否已使能
int32 current_position    # 当前位置 (原始值)
float32 current_angle     # 当前角度 (度, 0=竖直, 45=最大前倾)
int32 current_speed       # 当前速度设定值
bool alarm                # 是否有报警
bool position_reached     # 是否到达目标位置

# === 通信状态 ===
bool connected            # Modbus通信是否正常
