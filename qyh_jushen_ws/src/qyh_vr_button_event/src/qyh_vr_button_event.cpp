#include <sensor_msgs/msg/joy.hpp>
#include <cmath>
#include <qyh_lift_msgs/msg/lift_state.hpp>
#include <qyh_lift_msgs/srv/lift_control.hpp>
#include <rclcpp/rclcpp.hpp>
#include <qyh_waist_msgs/msg/waist_state.hpp>
#include <qyh_waist_msgs/srv/waist_control.hpp>
#include <qyh_standard_robot_msgs/msg/manual_velocity_command.hpp>
// #include <qyh_standard_robot_msgs/srv/control_start_manual_control.hpp>
// #include <qyh_standard_robot_msgs/srv/control_stop_manual_control.hpp>
#include <qyh_gripper_msgs/msg/gripper_state.hpp>
#include <qyh_gripper_msgs/srv/move_gripper.hpp>

namespace qyh_vr_button_event
{

using std::placeholders::_1;

class VRButtonEventNode : public rclcpp::Node
{
public:
  VRButtonEventNode()
  : Node("qyh_vr_button_event")
  {
    // parameters (topic/service names can be overridden)
    this->declare_parameter<std::string>("left_joy_topic", "/vr/left_controller/joy");
    this->declare_parameter<std::string>("right_joy_topic", "/vr/right_controller/joy");
    this->declare_parameter<std::string>("left_move_service", "/left/move_gripper");
    this->declare_parameter<std::string>("right_move_service", "/right/move_gripper");
    this->declare_parameter<std::string>("lift_control_service", "/lift/control");
    this->declare_parameter<std::string>("waist_control_service", "/waist/control");
    this->declare_parameter<std::string>("waist_state_topic", "/waist/state");
    this->declare_parameter<std::string>("manual_velocity_topic", "/manual_velocity_cmd");
    this->declare_parameter<double>("max_vx", 0.6);
    this->declare_parameter<double>("max_w", 1.2);
    this->declare_parameter<double>("deadzone", 0.2);
    this->declare_parameter<int>("joy_timeout_ms", 500);
    this->declare_parameter<int>("check_timer_ms", 100);
    this->declare_parameter<int>("chassis_keepalive_ms", 100);

    left_joy_topic_ = this->get_parameter("left_joy_topic").as_string();
    right_joy_topic_ = this->get_parameter("right_joy_topic").as_string();
    left_move_service_ = this->get_parameter("left_move_service").as_string();
    right_move_service_ = this->get_parameter("right_move_service").as_string();
    lift_control_service_ = this->get_parameter("lift_control_service").as_string();
    waist_control_service_ = this->get_parameter("waist_control_service").as_string();
    manual_velocity_topic_ = this->get_parameter("manual_velocity_topic").as_string();
    waist_state_topic_ = this->get_parameter("waist_state_topic").as_string();
    max_vx_ = this->get_parameter("max_vx").as_double();
    max_w_ = this->get_parameter("max_w").as_double();
    deadzone_ = this->get_parameter("deadzone").as_double();
    joy_timeout_ms_ = this->get_parameter("joy_timeout_ms").as_int();
    check_timer_ms_ = this->get_parameter("check_timer_ms").as_int();
    chassis_keepalive_ms_ = this->get_parameter("chassis_keepalive_ms").as_int();

    RCLCPP_INFO(this->get_logger(), "qyh_vr_button_event params: left_joy=%s right_joy=%s manual_vel=%s",
          left_joy_topic_.c_str(), right_joy_topic_.c_str(), manual_velocity_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "services: left_move=%s right_move=%s lift=%s waist=%s",
          left_move_service_.c_str(), right_move_service_.c_str(), lift_control_service_.c_str(), waist_control_service_.c_str());

    // subscribers
    left_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      left_joy_topic_, 10, std::bind(&VRButtonEventNode::leftJoyCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to left joy: %s", left_joy_topic_.c_str());

    right_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      right_joy_topic_, 10, std::bind(&VRButtonEventNode::rightJoyCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to right joy: %s", right_joy_topic_.c_str());

    // service clients
    left_move_client_ = this->create_client<qyh_gripper_msgs::srv::MoveGripper>(left_move_service_);
    right_move_client_ = this->create_client<qyh_gripper_msgs::srv::MoveGripper>(right_move_service_);
    lift_client_ = this->create_client<qyh_lift_msgs::srv::LiftControl>(lift_control_service_);
    waist_client_ = this->create_client<qyh_waist_msgs::srv::WaistControl>(waist_control_service_);

    RCLCPP_INFO(this->get_logger(), "Created service clients (may be unavailable until services come up)");

    // publisher for chassis manual velocity
    manual_vel_pub_ = this->create_publisher<qyh_standard_robot_msgs::msg::ManualVelocityCommand>(
      manual_velocity_topic_, 10);

    // waist state subscription to know current angle (used to "stop")
    waist_state_sub_ = this->create_subscription<qyh_waist_msgs::msg::WaistState>(
      waist_state_topic_, 10, [this](const qyh_waist_msgs::msg::WaistState::SharedPtr msg){
        waist_current_angle_ = msg->current_angle;
      });

    // initialize last joy times to now to avoid immediate timeout
    last_left_joy_time_ = this->now();
    last_right_joy_time_ = this->now();
    last_manual_publish_time_ = this->now();

    // timer to check joy timeouts and issue stops
    check_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(check_timer_ms_),
      std::bind(&VRButtonEventNode::checkTimeouts, this));

    RCLCPP_INFO(this->get_logger(), "qyh_vr_button_event node started");
  }

private:
  // Left controller: buttons X(bit2)/Y(bit3) -> open/close left gripper
  void leftJoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_left_joy_time_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "leftJoyCallback: buttons=%zu axes=%zu", msg->buttons.size(), msg->axes.size());
    // buttons vector length check
    std::vector<int> buttons = msg->buttons;

    bool x_pressed = false;
    bool y_pressed = false;
    if (buttons.size() > 2) x_pressed = buttons[2] != 0; // X
    if (buttons.size() > 3) y_pressed = buttons[3] != 0; // Y

    // edge detection
    // 只在按下和松开时发送命令
    if (x_pressed && !prev_left_x_) {
      RCLCPP_INFO(this->get_logger(), "Left X pressed -> opening left gripper");
      callMoveGripper(left_move_client_, 0, 255, 150); // open
    }
    if (!x_pressed && prev_left_x_) {
      RCLCPP_INFO(this->get_logger(), "Left X released -> stop left gripper");
      // 可选：松开时发送停止命令，若协议支持
    }
    if (y_pressed && !prev_left_y_) {
      RCLCPP_INFO(this->get_logger(), "Left Y pressed -> closing left gripper");
      callMoveGripper(left_move_client_, 255, 255, 150); // close
    }
    if (!y_pressed && prev_left_y_) {
      RCLCPP_INFO(this->get_logger(), "Left Y released -> stop left gripper");
      // 可选：松开时发送停止命令，若协议支持
    }
    prev_left_x_ = x_pressed;
    prev_left_y_ = y_pressed;

    // joystick axes: axes[0]=x (left/right), axes[1]=y (forward/back)
    double lx = 0.0, ly = 0.0;
    if (msg->axes.size() > 0) lx = msg->axes[0];
    if (msg->axes.size() > 1) ly = msg->axes[1];

    // apply deadzone separately for lift (ly) and waist (lx)
    if (std::abs(ly) < deadzone_) ly = 0.0;
    if (std::abs(lx) < deadzone_) lx = 0.0;

    RCLCPP_DEBUG(this->get_logger(), "Left joystick axes after deadzone: lx=%.3f ly=%.3f", lx, ly);

    handleLiftWithAxis(ly);
    handleWaistWithAxis(lx);
  }

  void rightJoyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    last_right_joy_time_ = this->now();
    RCLCPP_DEBUG(this->get_logger(), "rightJoyCallback: buttons=%zu axes=%zu", msg->buttons.size(), msg->axes.size());
    // buttons mapping: A(bit0), B(bit1)
    std::vector<int> buttons = msg->buttons;
    bool a_pressed = buttons.size() > 0 && buttons[0] != 0;
    bool b_pressed = buttons.size() > 1 && buttons[1] != 0;

    // 只在按下和松开时发送命令
    if (a_pressed && !prev_right_a_) {
      RCLCPP_INFO(this->get_logger(), "Right A pressed -> opening right gripper");
      callMoveGripper(right_move_client_, 0, 255, 150); // open
    }
    if (!a_pressed && prev_right_a_) {
      RCLCPP_INFO(this->get_logger(), "Right A released -> stop right gripper");
      // 可选：松开时发送停止命令，若协议支持
    }
    if (b_pressed && !prev_right_b_) {
      RCLCPP_INFO(this->get_logger(), "Right B pressed -> closing right gripper");
      callMoveGripper(right_move_client_, 255, 255, 150); // close
    }
    if (!b_pressed && prev_right_b_) {
      RCLCPP_INFO(this->get_logger(), "Right B released -> stop right gripper");
      // 可选：松开时发送停止命令，若协议支持
    }
    prev_right_a_ = a_pressed;
    prev_right_b_ = b_pressed;

    double rx = 0.0, ry = 0.0;
    if (msg->axes.size() > 0) rx = msg->axes[0];
    if (msg->axes.size() > 1) ry = msg->axes[1];

    // apply deadzone for chassis control
    if (std::abs(rx) < deadzone_) rx = 0.0;
    if (std::abs(ry) < deadzone_) ry = 0.0;

    RCLCPP_DEBUG(this->get_logger(), "Right joystick after deadzone: rx=%.3f ry=%.3f", rx, ry);

    // publish manual velocity command (map axes to vx,w)
    qyh_standard_robot_msgs::msg::ManualVelocityCommand mv;
    mv.vx = static_cast<double>(ry) * max_vx_;
    mv.w = static_cast<double>(rx) * max_w_;
    manual_vel_pub_->publish(mv);
    RCLCPP_DEBUG(this->get_logger(), "Published manual velocity vx=%.3f w=%.3f", mv.vx, mv.w);
    last_published_nonzero_manual_ = (std::abs(mv.vx) > 1e-6 || std::abs(mv.w) > 1e-6);
    // store last manual command and publish time for keepalive re-publishes
    last_manual_cmd_ = mv;
    last_manual_publish_time_ = this->now();
  }

  void callMoveGripper(
    rclcpp::Client<qyh_gripper_msgs::srv::MoveGripper>::SharedPtr client,
    uint8_t position, uint8_t speed, uint8_t force)
  {
    if (!client->wait_for_service(std::chrono::milliseconds(200))) {
      RCLCPP_WARN(this->get_logger(), "gripper move service not available: %s", client->get_service_name());
      return;
    }
    RCLCPP_DEBUG(this->get_logger(), "Calling MoveGripper: pos=%u speed=%u force=%u", position, speed, force);
    auto req = std::make_shared<qyh_gripper_msgs::srv::MoveGripper::Request>();
    req->position = position;
    req->speed = speed;
    req->force = force;
    auto fut = client->async_send_request(req,
      [this](rclcpp::Client<qyh_gripper_msgs::srv::MoveGripper>::SharedFuture response){
        if (!response.get()->success) {
          RCLCPP_WARN(this->get_logger(), "gripper move failed: %s", response.get()->message.c_str());
        }
      });
  }

  void handleLiftWithAxis(double ly)
  {
    // deadzone handled by caller
    // ly > deadzone => move up; ly < -deadzone => move down
    // 只在状态变化时发送
    if (ly > deadzone_) {
      if (!lift_holding_up_) {
        RCLCPP_INFO(this->get_logger(), "Lift joystick up: sending MOVE_UP hold");
        sendLiftCommand(5 /*CMD_MOVE_UP*/, 0.0f, true);
        lift_holding_up_ = true;
        lift_holding_down_ = false;
      }
    } else if (ly < -deadzone_) {
      if (!lift_holding_down_) {
        RCLCPP_INFO(this->get_logger(), "Lift joystick down: sending MOVE_DOWN hold");
        sendLiftCommand(6 /*CMD_MOVE_DOWN*/, 0.0f, true);
        lift_holding_down_ = true;
        lift_holding_up_ = false;
      }
    } else {
      if (lift_holding_up_) {
        RCLCPP_INFO(this->get_logger(), "Lift joystick released from up: cancelling hold");
        sendLiftCommand(5 /*CMD_MOVE_UP*/, 0.0f, false);
        lift_holding_up_ = false;
      }
      if (lift_holding_down_) {
        RCLCPP_INFO(this->get_logger(), "Lift joystick released from down: cancelling hold");
        sendLiftCommand(6 /*CMD_MOVE_DOWN*/, 0.0f, false);
        lift_holding_down_ = false;
      }
    }
  }

  void handleWaistWithAxis(double lx)
  {
    const double deadzone = 0.2;
    // Our waist does not support direction/stop commands — send absolute angles.
    // When leaning: send 45.0; when upright: send 0.0; to stop send current angle.
    double desired_angle;
    // 只在状态变化时发送
    if (lx < -deadzone) {
      if (!waist_leanning_) {
        desired_angle = 45.0; // lean forward
        RCLCPP_INFO(this->get_logger(), "Waist joystick left: lean forward (45 deg)");
        sendWaistCommand(5 /*CMD_GO_ANGLE*/, static_cast<float>(desired_angle), false);
        last_sent_waist_angle_ = desired_angle;
        waist_leanning_ = true;
      }
    } else if (lx > deadzone) {
      if (waist_leanning_ || last_sent_waist_angle_ != 0.0) {
        desired_angle = 0.0; // go upright
        RCLCPP_INFO(this->get_logger(), "Waist joystick right: upright (0 deg)");
        sendWaistCommand(5 /*CMD_GO_ANGLE*/, static_cast<float>(desired_angle), false);
        last_sent_waist_angle_ = desired_angle;
        waist_leanning_ = false;
      }
    } else {
      // 只有在waist_leanning_为true时才发送stop
      if (waist_leanning_) {
        desired_angle = waist_current_angle_;
        RCLCPP_INFO(this->get_logger(), "Waist joystick center: stop (current angle %.2f)", waist_current_angle_);
        sendWaistCommand(5 /*CMD_GO_ANGLE*/, static_cast<float>(desired_angle), false);
        last_sent_waist_angle_ = desired_angle;
        waist_leanning_ = false;
      }
    }
  }

  void sendLiftCommand(uint8_t command, float value, bool hold)
  {
    if (!lift_client_->wait_for_service(std::chrono::milliseconds(200))) return;
    RCLCPP_DEBUG(this->get_logger(), "Calling LiftControl: cmd=%u value=%.2f hold=%d", command, value, hold);
    auto req = std::make_shared<qyh_lift_msgs::srv::LiftControl::Request>();
    req->command = command;
    req->value = value;
    req->hold = hold;
    lift_client_->async_send_request(req,
      [this](rclcpp::Client<qyh_lift_msgs::srv::LiftControl>::SharedFuture res){
        if (!res.get()->success) {
          RCLCPP_WARN(this->get_logger(), "lift control failed: %s", res.get()->message.c_str());
        }
      });
  }

  void sendWaistCommand(uint8_t command, float value, bool hold)
  {
    if (!waist_client_->wait_for_service(std::chrono::milliseconds(200))) return;
    RCLCPP_DEBUG(this->get_logger(), "Calling WaistControl: cmd=%u value=%.2f hold=%d", command, value, hold);
    auto req = std::make_shared<qyh_waist_msgs::srv::WaistControl::Request>();
    req->command = command;
    req->value = value;
    req->hold = hold;
    waist_client_->async_send_request(req,
      [this](rclcpp::Client<qyh_waist_msgs::srv::WaistControl>::SharedFuture res){
        if (!res.get()->success) {
          RCLCPP_WARN(this->get_logger(), "waist control failed: %s", res.get()->message.c_str());
        }
      });
  }

  // params
  std::string left_joy_topic_, right_joy_topic_;
  std::string left_move_service_, right_move_service_;
  std::string lift_control_service_, waist_control_service_;
  std::string manual_velocity_topic_;
  double max_vx_, max_w_;
  std::string waist_state_topic_;
  double waist_current_angle_ = 0.0;
  double last_sent_waist_angle_ = -9999.0;
  double deadzone_ = 0.2;
  int joy_timeout_ms_ = 500;
  int check_timer_ms_ = 100;
  int chassis_keepalive_ms_ = 100;
  rclcpp::Time last_manual_publish_time_;
  qyh_standard_robot_msgs::msg::ManualVelocityCommand last_manual_cmd_;
  rclcpp::TimerBase::SharedPtr check_timer_;
  rclcpp::Time last_left_joy_time_;
  rclcpp::Time last_right_joy_time_;
  bool last_published_nonzero_manual_ = false;

  // subs/pubs/clients
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr left_joy_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr right_joy_sub_;
  rclcpp::Subscription<qyh_waist_msgs::msg::WaistState>::SharedPtr waist_state_sub_;
  rclcpp::Client<qyh_gripper_msgs::srv::MoveGripper>::SharedPtr left_move_client_;
  rclcpp::Client<qyh_gripper_msgs::srv::MoveGripper>::SharedPtr right_move_client_;
  rclcpp::Client<qyh_lift_msgs::srv::LiftControl>::SharedPtr lift_client_;
  rclcpp::Client<qyh_waist_msgs::srv::WaistControl>::SharedPtr waist_client_;
  rclcpp::Publisher<qyh_standard_robot_msgs::msg::ManualVelocityCommand>::SharedPtr manual_vel_pub_;

  // check timeouts and issue stops
  void checkTimeouts()
  {
    auto now = this->now();
    // right joystick (chassis): if timed out -> publish zero; else keepalive-republish every chassis_keepalive_ms_
    auto right_dt_ms = (now - last_right_joy_time_).nanoseconds() / 1000000;
    if (right_dt_ms > joy_timeout_ms_) {
      if (last_published_nonzero_manual_) {
        qyh_standard_robot_msgs::msg::ManualVelocityCommand mv;
        mv.vx = 0.0; mv.w = 0.0;
        manual_vel_pub_->publish(mv);
        last_published_nonzero_manual_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Right joystick timeout - publishing zero velocity");
      }
    } else {
      // not timed out: ensure we republish the last non-zero manual command at least every chassis_keepalive_ms_
      if (last_published_nonzero_manual_) {
        auto since_last_pub_ms = (now - last_manual_publish_time_).nanoseconds() / 1000000;
        if (since_last_pub_ms > chassis_keepalive_ms_) {
          manual_vel_pub_->publish(last_manual_cmd_);
          last_manual_publish_time_ = now;
          RCLCPP_DEBUG(this->get_logger(), "Re-published manual velocity keepalive");
        }
      }
    }

    // left joystick (lift/waist) timeout
    if ((now - last_left_joy_time_).nanoseconds() / 1000000 > joy_timeout_ms_) {
      // stop lift if it was holding
      if (lift_holding_up_ || lift_holding_down_) {
        sendLiftCommand(5 /*CMD_MOVE_UP*/, 0.0f, false);
        sendLiftCommand(6 /*CMD_MOVE_DOWN*/, 0.0f, false);
        lift_holding_up_ = lift_holding_down_ = false;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Left joystick timeout - stopping lift");
      }

      // stop waist if leaning
      if (waist_leanning_) {
        // send CMD_STOP to stop motion
        sendWaistCommand(9 /*CMD_STOP*/, 0.0f, false);
        waist_leanning_ = false;
        last_sent_waist_angle_ = -9999.0;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Left joystick timeout - stopping waist");
      }
    }
  }

  // state for edge detection and holds
  bool prev_left_x_ = false;
  bool prev_left_y_ = false;
  bool prev_right_a_ = false;
  bool prev_right_b_ = false;
  bool lift_holding_up_ = false;
  bool lift_holding_down_ = false;
  bool waist_leanning_ = false;
};

} // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<qyh_vr_button_event::VRButtonEventNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
