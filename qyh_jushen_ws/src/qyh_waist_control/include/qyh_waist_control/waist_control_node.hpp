/*
 * 腰部电机控制节点
 * 通过 Modbus TCP 与 PLC 通信
 * 控制机器人身体前倾角度 (0~45度)
 */

#ifndef QYH_WAIST_CONTROL__WAIST_CONTROL_NODE_HPP_
#define QYH_WAIST_CONTROL__WAIST_CONTROL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>
#include <mutex>

#include "qyh_waist_msgs/msg/waist_state.hpp"
#include "qyh_waist_msgs/srv/waist_control.hpp"
#include "modbus/modbus.hpp"

namespace qyh_waist_control
{

/**
 * Modbus 地址定义
 * 线圈地址从 2000 开始 (待定，先用这个占位)
 * 寄存器地址从 2000 开始 (待定，先用这个占位)
 * 
 * TODO: 根据实际PLC配置修改地址
 */
struct ModbusAddress
{
  // 线圈地址 (Coils) - 基地址 2000 (待修改)
  static constexpr int COIL_BASE = 2000;
  static constexpr int COIL_ENABLE = 0;           // 使能
  static constexpr int COIL_SET_ORIGIN = 1;       // 设置原点 (不使用)
  static constexpr int COIL_LEAN_FORWARD = 2;     // 正转(前倾)
  static constexpr int COIL_LEAN_BACK = 3;        // 反转(后仰/回正)
  static constexpr int COIL_RESET = 4;            // 复位
  static constexpr int COIL_ALARM = 5;            // 报警 (PLC通知)
  static constexpr int COIL_GO_POSITION = 10;     // 绝对位置GO
  static constexpr int COIL_POSITION_REACHED = 20; // 绝对位置到达 (PLC通知)

  // 寄存器地址 (Holding Registers) - 基地址 2000 (待修改)
  static constexpr int REG_BASE = 2000;
  static constexpr int REG_TARGET_POSITION = 0;   // 目标绝对位置 (int32, 2个寄存器)
  static constexpr int REG_SPEED = 20;            // 绝对速度 (int32, 2个寄存器)
  static constexpr int REG_CURRENT_POSITION = 40; // 当前绝对位置 (int32, 2个寄存器)
};

/**
 * 位置常量
 * 163711 = 前倾45度
 * 230715 = 竖直0度
 */
struct PositionConstants
{
  static constexpr int32_t UPRIGHT = 230715;      // 竖直位置
  static constexpr int32_t MAX_LEAN = 163711;     // 最大前倾位置
  static constexpr float MAX_ANGLE = 45.0f;       // 最大前倾角度
};

class WaistControlNode : public rclcpp::Node
{
public:
  explicit WaistControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~WaistControlNode();

private:
  // Modbus 连接
  bool connect_modbus();
  void disconnect_modbus();

  // 定时器回调 - 读取状态并发布
  void timer_callback();

  // 重连定时器回调
  void reconnect_callback();

  // 读取状态
  bool read_waist_state();

  // 线圈操作
  bool write_coil(int offset, bool value);
  bool read_coil(int offset);

  // 寄存器操作 (int32)
  bool write_register_int32(int offset, int32_t value);
  int32_t read_register_int32(int offset);

  // 位置和角度转换
  int32_t angle_to_position(float angle);
  float position_to_angle(int32_t position);

  // 控制功能
  bool enable_waist(bool enable);
  bool set_speed(int32_t speed);
  bool go_to_position(int32_t position);
  bool go_to_angle(float angle);
  bool manual_lean(bool forward, bool hold);
  bool reset_alarm();
  bool stop_move();
  bool go_upright();

  // 服务回调
  void handle_control(
    const qyh_waist_msgs::srv::WaistControl::Request::SharedPtr request,
    qyh_waist_msgs::srv::WaistControl::Response::SharedPtr response);

  // 参数
  std::string plc_ip_;
  int plc_port_;
  double publish_rate_;
  int32_t position_upright_;
  int32_t position_max_lean_;
  float max_angle_;

  // Modbus 连接
  std::unique_ptr<modbus::ModbusTCP> modbus_ctx_;
  std::mutex modbus_mutex_;
  bool is_connected_;

  // 状态
  qyh_waist_msgs::msg::WaistState current_state_;
  bool is_enabled_;

  // ROS2 接口
  rclcpp::Publisher<qyh_waist_msgs::msg::WaistState>::SharedPtr state_pub_;
  rclcpp::Service<qyh_waist_msgs::srv::WaistControl>::SharedPtr control_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

}  // namespace qyh_waist_control

#endif  // QYH_WAIST_CONTROL__WAIST_CONTROL_NODE_HPP_
