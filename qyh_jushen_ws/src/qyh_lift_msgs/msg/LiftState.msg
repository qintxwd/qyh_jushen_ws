# 升降电机状态消息
# Modbus RTU: Slave ID=1, Coils从1000, Registers从1000

std_msgs/Header header

# === 状态显示 ===
bool enabled              # 是否已使能
float32 current_position  # 当前高度位置
float32 current_speed     # 当前速度设定值
bool alarm                # 是否有报警
bool position_reached     # 是否到达目标位置

# === 通信状态 ===
bool connected            # Modbus通信是否正常
