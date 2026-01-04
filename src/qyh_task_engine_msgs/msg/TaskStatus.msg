# 任务状态消息
# 描述整个任务的执行状态

std_msgs/Header header

# 任务唯一标识
string task_id

# 任务名称
string task_name

# 任务状态: idle, running, paused, success, failure, cancelled
string status

# 当前正在执行的节点 ID
string current_node_id

# 已完成的节点数
uint32 completed_nodes

# 总节点数
uint32 total_nodes

# 执行进度 (0.0 - 1.0)
float32 progress

# 状态消息
string message

# 开始时间戳
builtin_interfaces/Time start_time

# 已执行时间 (秒)
float64 elapsed_time
