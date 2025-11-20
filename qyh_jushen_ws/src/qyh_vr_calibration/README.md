# VR 标定包 (VR Calibration Package)

本功能包用于管理不同用户的 VR 标定配置文件。
数据存储在 `~/qyh_jushen_ws/persistent/vr_calibration/<username>.yaml` 路径下。

## 服务列表 (Services)

- `vr_calibration/list_profiles` (qyh_vr_calibration_msgs/srv/ListProfiles)
  - 列出所有用户的配置。
- `vr_calibration/get_profile` (qyh_vr_calibration_msgs/srv/GetProfile)
  - 获取指定用户的详细配置。
- `vr_calibration/update_action_sample` (qyh_vr_calibration_msgs/srv/UpdateActionSample)
  - 更新或添加单个动作的采样点。
- `vr_calibration/delete_action_samples` (qyh_vr_calibration_msgs/srv/DeleteActionSamples)
  - 删除指定动作的采样点（或删除某用户的所有采样）。
- `vr_calibration/delete_profile` (qyh_vr_calibration_msgs/srv/DeleteProfile)
  - 删除指定用户的整个配置文件。

## 采样索引映射 (Sample Index Mapping)

`VRActionSample` 中的 `sample_index` 对应特定的标定姿态。
请参考下表进行映射：

| 采样索引 (Sample Index) | 动作描述 (Action Description) |
| :--- | :--- |
| 0 | T-Pose (双臂水平伸展) |
| 1 | Arms forward (双臂前平举 90 度) |
| 2 | Arms up (双臂上举 180 度) |
| 3 | Arms down (双臂自然下垂) |
