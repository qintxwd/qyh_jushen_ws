/*
 * 关机控制节点实现
 */

#include "modbus/modbus.hpp"
#include <chrono>
#include <cstring>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
namespace qyh_shutdown
{

class QyhShutdownNode : public rclcpp::Node
{
public:
  explicit QyhShutdownNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~QyhShutdownNode();

private:
  // Modbus 连接
  bool connect_modbus();
  void disconnect_modbus();

  // 定时器回调 - 读取状态并发布
  void timer_callback();

  // 重连定时器回调
  void reconnect_callback();

  // 读取状态
  bool read_shutdown_state();

  // 线圈操作
  bool write_coil(int offset, bool value);
  // 控制功能
  bool request_shutdown();         // 请求系统关机
  void execute_system_shutdown();  // 执行系统关机命令

  // 服务回调
  void handle_control(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  // 参数
  std::string plc_ip_;
  int plc_port_;
  double publish_rate_;

  // Modbus 连接
  std::unique_ptr<modbus::ModbusTCP> modbus_ctx_;
  std::mutex modbus_mutex_;
  bool is_connected_;

  // 状态
  std_msgs::msg::Bool current_state_;
  bool shutdown_requested_;    // 是否收到关机请求（硬件按钮或软件命令）
  bool shutdown_in_progress_;  // 关机是否正在进行中

  // ROS2 接口
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr state_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr control_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;  // 重连定时器
};

QyhShutdownNode::QyhShutdownNode(const rclcpp::NodeOptions &options)
  : Node("qyh_shutdown_node", options)
  , modbus_ctx_(nullptr)
  , is_connected_(false)
  , shutdown_requested_(false)
  , shutdown_in_progress_(false)
{
  // 声明参数
  this->declare_parameter<std::string>("plc_ip", "192.168.1.88");
  this->declare_parameter<int>("plc_port", 502);
  this->declare_parameter<double>("publish_rate", 10.0);

  // 获取参数
  plc_ip_ = this->get_parameter("plc_ip").as_string();
  plc_port_ = this->get_parameter("plc_port").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();

  RCLCPP_INFO(this->get_logger(), "Starting qyh_shutdown_node");
  RCLCPP_INFO(this->get_logger(), "PLC IP: %s, Port: %d", plc_ip_.c_str(), plc_port_);

  // 连接 Modbus TCP
  if (!connect_modbus())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to PLC");
  }

  // 创建发布者
  state_pub_ = this->create_publisher<std_msgs::msg::Bool>("shutdown_state", 10);

  // 创建服务
  control_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "qyh_shutdown", std::bind(&QyhShutdownNode::handle_control, this, std::placeholders::_1, std::placeholders::_2));

  // 创建定时器 - 状态发布
  auto timer_period = std::chrono::duration<double>(1.0);
  timer_ = this->create_wall_timer(std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
                                   std::bind(&QyhShutdownNode::timer_callback, this));

  // 创建重连定时器 - 10秒间隔
  reconnect_timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&QyhShutdownNode::reconnect_callback, this));

  RCLCPP_INFO(this->get_logger(), "QyhShutdown Control Node started");
}

QyhShutdownNode::~QyhShutdownNode()
{
  disconnect_modbus();
}

bool QyhShutdownNode::connect_modbus()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  try
  {
    modbus_ctx_ = std::make_unique<modbus::ModbusTCP>(plc_ip_, plc_port_);
    modbus_ctx_->set_response_timeout(1, 0);  // 1秒超时
    modbus_ctx_->connect();

    RCLCPP_INFO(this->get_logger(), "Successfully connected to PLC at %s:%d", plc_ip_.c_str(), plc_port_);
    is_connected_ = true;
    return true;
  }
  catch (const modbus::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Modbus connection failed: %s", e.what());
    modbus_ctx_.reset();
    is_connected_ = false;
    return false;
  }
}

void QyhShutdownNode::disconnect_modbus()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (modbus_ctx_)
  {
    try
    {
      modbus_ctx_->close();
    }
    catch (...)
    {
      // 忽略关闭错误
    }
    modbus_ctx_.reset();
    is_connected_ = false;
    RCLCPP_INFO(this->get_logger(), "Disconnected from PLC");
  }
}

void QyhShutdownNode::timer_callback()
{
  current_state_.data = false;
  if (is_connected_ && read_shutdown_state())
  {
  }
  state_pub_->publish(current_state_);
}

void QyhShutdownNode::reconnect_callback()
{
  // 已连接时不需要重连
  if (is_connected_)
  {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Attempting to reconnect to PLC...");
  if (connect_modbus())
  {
    RCLCPP_INFO(this->get_logger(), "Reconnected to PLC successfully");
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Reconnection failed, will retry in 10 seconds");
  }
}

bool QyhShutdownNode::read_shutdown_state()
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_)
  {
    return false;
  }

  try
  {
    // 读取线圈状态 (地址100)
    auto coils = modbus_ctx_->read_coils(100, 1);

    current_state_.data = (coils[0] != 0);

    // 如果检测到硬件关机请求且还没有开始关机流程
    if (current_state_.data && !shutdown_in_progress_)
    {
      RCLCPP_WARN(this->get_logger(), "Hardware shutdown button pressed! Initiating system shutdown...");
      shutdown_in_progress_ = true;
      // 在新线程中执行关机，避免阻塞状态读取
      std::thread(&QyhShutdownNode::execute_system_shutdown, this).detach();
    }
    return true;
  }
  catch (const modbus::Exception &e)
  {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to read shutdown state: %s", e.what());
    is_connected_ = false;
    return false;
  }
}

bool QyhShutdownNode::write_coil(int offset, bool value)
{
  std::lock_guard<std::mutex> lock(modbus_mutex_);

  if (!modbus_ctx_)
  {
    return false;
  }

  try
  {
    modbus_ctx_->write_coil(offset, value);
    return true;
  }
  catch (const modbus::Exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to write coil %d: %s", offset, e.what());
    return false;
  }
}

bool QyhShutdownNode::request_shutdown()
{
  RCLCPP_WARN(this->get_logger(), "Software shutdown requested!");

  if (shutdown_in_progress_)
  {
    RCLCPP_INFO(this->get_logger(), "Shutdown already in progress");
    return true;
  }

  // 写关机线圈通知PLC
  if (!write_coil(100, true))
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to write shutdown coil to PLC");
    return false;
  }

  shutdown_requested_ = true;
  shutdown_in_progress_ = true;

  // 在新线程中执行关机
  std::thread(&QyhShutdownNode::execute_system_shutdown, this).detach();

  return true;
}

void QyhShutdownNode::execute_system_shutdown()
{
  RCLCPP_WARN(this->get_logger(), "Executing system shutdown in 3 seconds...");

  // 等待3秒让Web页面有时间显示关机提示
  std::this_thread::sleep_for(std::chrono::seconds(3));

  RCLCPP_WARN(this->get_logger(), "System shutdown NOW!");

  // 执行系统关机命令
  int result = std::system("sudo shutdown -h now");

  if (result != 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to execute shutdown command, code: %d", result);
  }
}

void QyhShutdownNode::handle_control(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                     std_srvs::srv::Trigger::Response::SharedPtr response)
{
  request_shutdown();
}

}  // namespace qyh_shutdown

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<qyh_shutdown::QyhShutdownNode>();

  RCLCPP_INFO(node->get_logger(), "Qyh Shutdown Control Node is running...");

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
