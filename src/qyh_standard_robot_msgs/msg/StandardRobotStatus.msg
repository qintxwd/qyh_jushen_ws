# 是否触发急停 (10001)
bool is_emergency_stopped
# 急停是否可恢复 (10002)
bool is_emergency_recoverable
# 是否解抱闸 (10003)
bool is_brake_released
# 是否正在充电 (10004)
bool is_charging
# 是否低功耗模式 (10005)
bool is_low_power_mode
# 遇障减速 (10006)
bool obstacle_slowdown
# 遇障暂停 (10007)
bool obstacle_paused
# 能否运行移动任务 (10009)
bool can_run_motion_task
# 是否处于自动模式 (10051)
bool is_auto_mode
# 车辆载荷，true是有载荷，false为空载 (10054)
bool is_loaded
# 是否有无限网卡 (10055)
bool has_wifi
# 工位避障信号,true是工位有障碍，false是无障碍 (10136)
bool workplace_obstacle_signal

# 0x01: 系统正在初始化 
# 0x02: 系统空闲 
# 0x03: 系统出错 
# 0x04: 正在启动定位 
# 0x05: 导航正在初始化 
# 0x06: 导航正在寻路 
# 0x07: 正在等待到达目标位置 
# 0x08: 检测到障碍 , 减速 
# 0x09: 导航正在重新寻路 
# 0x0A: 遇到障碍暂停运动 
# 0x0B: 无法抵达目标位置 
# 0x0E: 正在初始化执行固定路径 
# 0x0F: 正在等待固定路径执行结束 
# 0x10: 执行固定路径检测到障碍 , 减速前进 
# 0x11: 执行固定路径遇到障碍暂停运动 
# 0x12: 无法检测到目标站点 
# 0x13: 执行固定路径时用戶暂停了 
# 0x15: 导航过程中出错，如：偏离路径 
# 0x16: 硬件错误
# 以上是我们的系统状态的枚举值，每个值都有对应的描述
uint16 SYS_STATUS_INITIALIZING = 0x01
uint16 SYS_STATUS_IDLE = 0x02
uint16 SYS_STATUS_ERROR = 0x03
uint16 SYS_STATUS_LOCATING = 0x04
uint16 SYS_STATUS_NAV_INITIALIZING = 0x05
uint16 SYS_STATUS_NAV_PATHFINDING = 0x06
uint16 SYS_STATUS_WAITING_ARRIVAL = 0x07
uint16 SYS_STATUS_OBSTACLE_DETECTED = 0x08
uint16 SYS_STATUS_NAV_REPATHING = 0x09
uint16 SYS_STATUS_OBSTACLE_PAUSED = 0x0A
uint16 SYS_STATUS_CANNOT_ARRIVE = 0x0B
uint16 SYS_STATUS_FIXED_PATH_INITIALIZING = 0x0E
uint16 SYS_STATUS_WAITING_FIXED_PATH_END = 0x0F
uint16 SYS_STATUS_FIXED_PATH_OBSTACLE_DETECTED = 0x10
uint16 SYS_STATUS_FIXED_PATH_OBSTACLE_PAUSED = 0x11
uint16 SYS_STATUS_CANNOT_DETECT_TARGET = 0x12
uint16 SYS_STATUS_USER_PAUSED_FIXED_PATH = 0x13
uint16 SYS_STATUS_NAV_ERROR = 0x15
uint16 SYS_STATUS_HARDWARE_ERROR = 0x16

uint16 system_status

# 30002  定位状态  enum/uint16
# 0x01: 定位未启动 
# 0x02: 定位正在初始化 
# 0x03: 定位成功 
# 0x04: 正在重定位 
# 0x05: 定位错误，需要重新启动定位
# 以上是定位状态的枚举值，每个值都有对应的描述
uint16 LOCATION_STATUS_UNSTARTED = 0x01
uint16 LOCATION_STATUS_INITIALIZING = 0x02
uint16 LOCATION_STATUS_SUCCESS = 0x03
uint16 LOCATION_STATUS_RELOCATING = 0x04
uint16 LOCATION_STATUS_ERROR = 0x05

uint16 location_status

# 30003  位姿 x  int32  mm 
# 30005  位姿 y  int32 
# 30007  位姿 yaw  int32  rad*1000
geometry_msgs/PoseWithCovarianceStamped pose

# 30009  位姿置信度  uint16  0.01%
float32 pose_confidence

# 30015  当前站点编号  uint16
uint16 current_station_id

# 30016  操作状态  enum/uint16 
# 0x00 状态不可用 
# 0x01 自动控制模式 
# 0x02 手动控制模式 
uint16 OPERATION_STATUS_UNAVAILABLE = 0x00
uint16 OPERATION_STATUS_AUTO_CONTROL = 0x01
uint16 OPERATION_STATUS_MANUAL_CONTROL = 0x02

uint16 operation_status

# 30017  x 方向线速度  int16  mm/s 
# 30018  y 方向线速度  int16 
# 30019  ⻆速度  int16  (1/1000)rads/s
geometry_msgs/Twist twist


# 30020  调度模式  enum/uint16 
# 0x01 手动模式 
# 0x02 自动模式 
# 0x03 维保模式
uint16 SCHEDULING_MODE_MANUAL = 0x01
uint16 SCHEDULING_MODE_AUTO = 0x02
uint16 SCHEDULING_MODE_MAINTENANCE = 0x03

uint16 scheduling_mode

# 30027  系统上一次错误  uint32
uint32 last_error_code

# 30033  电池电压  uint16  mV 
float32 battery_voltage
# 30034  电池电流  int16  负数为充电电流  mA 
float32 battery_current
# 30035  电池温度  int16  °C 
float32 battery_temperature
# 30036  电池预计使用时间  uint16  min 
float32 battery_estimated_using_time
# 30037  当前电量百分比  uint16  [1, 100]
float32 battery_remaining_percentage

# 30038  当前电池的状态  enum/uint16 
# 0x00: 无效状态 
# 0x02: 正在充电 
# 0x03: 未充电
uint16 BATTERY_STATUS_INVALID = 0x00
uint16 BATTERY_STATUS_CHARGING = 0x02
uint16 BATTERY_STATUS_NOT_CHARGING = 0x03

uint16 battery_status

# 30039  电池循环次数  uint16  电池充放电循环使用了的次数 
uint16 battery_cycle_count

# 30040  电池标称容量  uint16  mAh 
uint16 battery_nominal_capacity

# 30041  运动总里程  uint32  m 
uint32 total_motion_distance

# 30043  开机总时间  uint32  s 
uint32 total_boot_time

# 30045  开机总次数  uint32  次 
uint32 total_boot_count

# 30047  系统当前时间  uint32  Linux 时间戳
uint32 system_current_time

# [30049,30052]  IP.1~4  uint16  对外通信IP地址，比如192.168.71.50,
uint16[4] ip_addresses

# 30065  当前地图名的前两个字节编码 
# uint16  比如当前地图名为： “1aa”, 那么此寄存器的值为： 0x3161
string current_map_name  # 这里说明一下，当前地图名的前两个字节编码，不是地图名的全部编码

# 30070  当前系统音量  uint16  1-100 
uint16 current_system_volume

# 30071  关机状态  enum/uint16 
# 0x00: 无效 0x01: ⻋辆即将关机 
uint16 SHUTDOWN_STATUS_INVALID = 0x00
uint16 SHUTDOWN_STATUS_SHUTTING_DOWN = 0x01

uint16 shutdown_status



# 30072  当前时区  int16  系统所在时区 
int16 current_timezone

# 30073  机器人运动状态  enum/uint16 
# 0x00: 未知状态 
# 0x01: 静止 
# 0x02: 前进 
# 0x03: 后退 
# 0x04: 左转 
# 0x05: 右转 
# 0x06: 原地左转 
# 0x07: 原地右转 
# 0x08: 左移 
# 0x09: 右移 
uint16 MOTION_STATUS_UNKNOWN = 0x00
uint16 MOTION_STATUS_STOPPED = 0x01
uint16 MOTION_STATUS_FORWARD = 0x02
uint16 MOTION_STATUS_BACKWARD = 0x03
uint16 MOTION_STATUS_LEFT = 0x04
uint16 MOTION_STATUS_RIGHT = 0x05
uint16 MOTION_STATUS_IN_PLACE_LEFT = 0x06
uint16 MOTION_STATUS_IN_PLACE_RIGHT = 0x07
uint16 MOTION_STATUS_LEFT_SHIFT = 0x08
uint16 MOTION_STATUS_RIGHT_SHIFT = 0x09

uint16 motion_status

# 30091  工位避障触发源  uint32  按bit位分布 
# 0x01 主雷达 
# 0x02 辅雷达 
# 0x04 深度相机1 
# 0x08 深度相机2 
# 0x10 深度相机3 
# 0x20 深度相机4 
# 0x40 避障雷达1 
# 0x80 避障雷达2 
# 0x100 避障雷达3 
# 0x200 避障雷达4
bool obstacle_avoidance_triggered_main_radar
bool obstacle_avoidance_triggered_aux_radar
bool obstacle_avoidance_triggered_depth_camera1
bool obstacle_avoidance_triggered_depth_camera2
bool obstacle_avoidance_triggered_depth_camera3
bool obstacle_avoidance_triggered_depth_camera4
bool obstacle_avoidance_triggered_obstacle_radar1
bool obstacle_avoidance_triggered_obstacle_radar2
bool obstacle_avoidance_triggered_obstacle_radar3
bool obstacle_avoidance_triggered_obstacle_radar4

# 30097  当前正在运行的 mission id uint32 
uint32 current_mission_id

# 30099  Mission 运行状态  enum/uint16 
# 0x00: 无效状态 
# 0x02: 任务在队列中，但是又还没有启动的状态 
# 0x03: 正在执行 
# 0x04: 暂停执行 
# 0x05: 执行结束 
# 0x06: 正在取消 
uint16 MISSION_STATUS_INVALID = 0x00
uint16 MISSION_STATUS_QUEUED = 0x02
uint16 MISSION_STATUS_EXECUTING = 0x03
uint16 MISSION_STATUS_PAUSED = 0x04
uint16 MISSION_STATUS_FINISHED = 0x05
uint16 MISSION_STATUS_CANCELING = 0x06

uint16 mission_status



# 30100  Mission 执行结果  enum/uint16 
# 0x00: 无效状态 
# 0x01: 任务执行成功 
# 0x02: 任务取消 
# 0x03: 任务执行出错 
uint16 MISSION_RESULT_INVALID = 0x00
uint16 MISSION_RESULT_SUCCESS = 0x01
uint16 MISSION_RESULT_CANCELED = 0x02
uint16 MISSION_RESULT_ERROR = 0x03

uint16 mission_result

# 30101  Mission 错误码  uint32
uint32 mission_error_code

# 30113  移动任务状态  enum/uint16 
# 0x00: 无效状态 
# 0x02: 等待开始执行 
# 0x03: 正在执行 
# 0x04: 暂停执行 
# 0x05: 执行结束 
# 0x06: 正在取消 
# 0x08: 交通管制 
uint16 MOVE_TASK_STATUS_INVALID = 0x00
uint16 MOVE_TASK_STATUS_QUEUED = 0x02
uint16 MOVE_TASK_STATUS_EXECUTING = 0x03
uint16 MOVE_TASK_STATUS_PAUSED = 0x04
uint16 MOVE_TASK_STATUS_FINISHED = 0x05
uint16 MOVE_TASK_STATUS_CANCELING = 0x06
uint16 MOVE_TASK_STATUS_TRAFFIC_CONTROL = 0x08

uint16 move_task_status



# 30114  当前移动任务 no  int32 
int32 current_move_task_no
# 30116  当前移动任务目标站点 uint16 
uint16 current_move_task_destination_station

# 30117  当前路径编号，移动任务运行过程中有效 uint16 
uint16 current_move_task_path_no
# 30118  当前移动任务起始站点 uint16 
uint16 current_move_task_start_station


# 30122  移动任务结果  enum/uint16 
# 0x00: 无效状态 
# 0x01: 任务执行成功 
# 0x02: 任务取消 
# 0x03: 任务执行出错
uint16 MOVE_TASK_RESULT_INVALID = 0x00
uint16 MOVE_TASK_RESULT_SUCCESS = 0x01
uint16 MOVE_TASK_RESULT_CANCELED = 0x02
uint16 MOVE_TASK_RESULT_ERROR = 0x03

uint16 move_task_result


