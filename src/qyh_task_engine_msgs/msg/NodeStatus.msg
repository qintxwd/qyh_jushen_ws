# 节点状态消息
# 描述单个行为树节点的执行状态

# 节点唯一标识
string node_id

# 节点类型 (如: ArmMoveJ, GripperControl, Sequence 等)
string node_type

# 节点名称 (可选的显示名称)
string node_name

# 节点状态: idle, running, success, failure, halted
string status

# 状态消息
string message

# 执行开始时间戳
builtin_interfaces/Time start_time

# 执行耗时 (秒)
float64 duration
