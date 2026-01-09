# 关机状态消息

# 是否正在关机
bool shutdown_in_progress

# 触发来源: 0=无, 1=硬件按钮, 2=软件命令
uint8 trigger_source

# 常量定义
uint8 SOURCE_NONE = 0
uint8 SOURCE_HARDWARE = 1
uint8 SOURCE_SOFTWARE = 2

# 倒计时秒数 (关机前剩余时间)
int32 countdown_seconds

# PLC连接状态
bool plc_connected
