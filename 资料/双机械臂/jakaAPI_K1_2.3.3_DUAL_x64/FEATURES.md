# 特性说明

## 支持机型
* CAN版本双臂
  
## 支持语言
* C++

## 支持系统
* x64 >= ubuntu20
* nvidia orin >= ubuntu20

## 接口与特性
> 仅针对C++接口，在x64和nvidia orin上
> 下列未提及的接口，未做确认

接口特性支持情况说明。以及纳入自动化测试和常规发包测试流程的接口说明

### 会话管理
| 接口与特性         | 测试覆盖率 | 功能覆盖率 | 备注 |
| ------------------ | ---------- | ---------- | ---- |
| login_in           | 100        | 100        |      |
| login_out          | 100        | 100        |      |
| 单主机连接多机器人 | 0          | 0          |      |
| 多主机连接单机器人 | 0          | 0          |      |

### 状态监控

| 接口名称               | 测试覆盖率 | 功能覆盖率 | 备注                                      |
| ---------------------- | ---------- | ---------- | ----------------------------------------- |
| power_on               | 100        | 100        |                                           |
| power_off              | 100        | 100        |                                           |
| enable_robot           | 100        | 100        |                                           |
| disable_robot          | 100        | 100        |                                           |
| get_robot_state        | 100        | 100        |                                           |
| robot_is_inpos         | 0          | 0          |                                           |
| get_sdk_version        | 0          | 100        |                                           |
| robot_get_default_base | 100        | 100        | >=3.0.6_robohub7,否则获取的姿态数据不正确 |
| robot_set_default_base | 100        | 100        | >=3.0.6_robohub7                          |



### 运动
| 接口名称                       | 测试覆盖率 | 功能覆盖率 | 备注             |
| ------------------------------ | ---------- | ---------- | ---------------- |
| robot_run_multi_movj           | 50         | 100        |                  |
| robot_run_multi_movj.is_block  | 0          | 100        |                  |
| robot_run_multi_movj.move_mode | 100        | 100        |                  |
| robot_run_multi_movj.tol       | 0          | 50         | >=3.0.6_robohub7 |
| robot_run_multi_movl           | 50         | 100        |                  |
| robot_run_multi_movl.is_block  | 0          | 100        |                  |
| robot_run_multi_movl.move_mode | 100        | 100        |                  |
| robot_run_multi_movl.tol       | 0          | 50         | >=3.0.6_robohub7 |
| motion_abort                   | 0          | 100        |                  |
| kine_inverse                   | 0          | 100        |                  |
| kine_forward                   | 0          | 100        |                  |

### 安全

### EDG与servo运动
| 接口名称                   | 测试覆盖率 | 功能覆盖率 | 备注             |
| -------------------------- | ---------- | ---------- | ---------------- |
| edg_send                   | 100        | 100        |                  |
| edg_get_stat               | 100        | 100        |                  |
| edg_get_stat.jpos          | 100        | 100        |                  |
| edg_get_stat.pose          | 100        | 100        |                  |
| edg_get_stat.sensor_torque | 100        | 100        | >=3.0.6_robohub7 |
| edg_servo_j                | 80         | 100        |                  |
| edg_servo_j.step_num       | 0          | 100        |                  |
| edg_servo_j.move_mode      | 0          | 100        |                  |
| edg_servo_p                | 100        | 100        |                  |
| edg_servo_p.step_num       | 0          | 100        |                  |
| edg_servo_p.move_mode      | 0          | 100        |                  |
| edg_stat_details           | 100        | 100        |                  |
| edg_stat_free_key          | 100        | 100        |                  |
| edg_recv                   | 100        | 100        |                  |
| servo_move_enable          | 100        | 100        |                  |
| servo_move_use_none_filter | 100        | 100        |                  |
| servo_move_use_joint_LPF   | 0          | 100        |                  |
| servo_move_use_joint_NLF   | 0          | 100        |                  |
| servo_move_use_carte_NLF   | 0          | 100        |                  |
| servo_move_use_joint_MMF   | 0          | 100        |                  |
| servo_speed_foresight      | 0          | 100        |                  |