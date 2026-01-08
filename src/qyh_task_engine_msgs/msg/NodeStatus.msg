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

# ========== 扩展字段（用于复杂场景）==========

# 子节点数量 (仅对复合节点有效)
uint32 children_count

# 当前子节点索引 (仅对 Sequence/Selector 有效)
uint32 current_child_index

# 当前循环次数 (仅对 Loop 有效，从 0 开始)
uint32 current_iteration

# 总循环次数 (仅对 Loop 有效，0 表示无限循环)
uint32 total_iterations
