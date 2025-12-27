#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>

class QyhTeleopNode : public rclcpp::Node {
public:
    QyhTeleopNode() : Node("qyh_teleop") {
        // Publishers
        left_servo_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/teleop/left/servo_p", 10);

        // Subscribers
        left_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vr/left_controller/pose", 10, std::bind(&QyhTeleopNode::vr_left_pose_callback, this, std::placeholders::_1));
        left_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/vr/left_controller/joy", 10, std::bind(&QyhTeleopNode::left_joy_callback, this, std::placeholders::_1));
        left_tcp_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/left_arm/tcp_pose", 10, std::bind(&QyhTeleopNode::left_tcp_pose_callback, this, std::placeholders::_1));

        // Clutch thresholds
        grip_engage_ = 0.6;
        grip_release_ = 0.3;

        // State machine
        left_clutch_state_ = "IDLE";

        // Current values
        left_grip_value_ = 0.0;
        cur_tcp_position_ = {0.0, 0.0, 0.0};
        cur_tcp_rotm_ = Eigen::Matrix3d::Identity();
        tcp_ready_ = false;

        // Transforms
        T_base_base_left_ = make_transform_matrix({0, 0, 0}, {0, 0, -M_PI / 6});
        T_lt_forward_ = make_transform_matrix({0, 0, 0}, {0, -M_PI / 2, -M_PI / 2});
        T_vr_human_align_ = make_transform_matrix({0, 0, 0}, {0, -M_PI * 35 / 180, 0});  // -35 degrees

        // Incremental mode cache
        T_vr_start_base_.reset();
        T_tcp_start_forward_.reset();

        last_rotation_.reset();

        RCLCPP_INFO(this->get_logger(), "qyh_teleop node started");
    }

private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr left_servo_pub_;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr left_joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr left_tcp_sub_;

    // Clutch thresholds
    double grip_engage_;
    double grip_release_;

    // State machine
    std::string left_clutch_state_;

    // Current values
    double left_grip_value_;
    std::vector<double> cur_tcp_position_;
    Eigen::Matrix3d cur_tcp_rotm_;
    bool tcp_ready_;

    // Transforms
    Eigen::Affine3d T_base_base_left_;
    Eigen::Affine3d T_lt_forward_;
    Eigen::Affine3d T_vr_human_align_;

    // Incremental mode cache
    std::optional<Eigen::Affine3d> T_vr_start_base_;
    std::optional<Eigen::Affine3d> T_tcp_start_forward_;

    std::optional<Eigen::Quaterniond> last_rotation_;

    // Helper functions
    Eigen::Affine3d make_transform_matrix(const std::vector<double>& translation, const std::vector<double>& rpy) {
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        Eigen::Matrix3d R;
        // SciPy 'xyz' extrinsic: Rz * Ry * Rx
        R = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
        T.linear() = R;
        T.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
        return T;
    }

    Eigen::Affine3d invert_transform(const Eigen::Affine3d& T) {
        return T.inverse();
    }

    void print_pose(const Eigen::Affine3d& T, const std::string& label) {
        Eigen::Vector3d pos = T.translation();
        Eigen::Matrix3d rot = T.linear();
        Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2) * 180 / M_PI;  // degrees
        RCLCPP_INFO(this->get_logger(), "%s Position: x=%.3f, y=%.3f, z=%.3f", label.c_str(), pos.x(), pos.y(), pos.z());
        RCLCPP_INFO(this->get_logger(), "%s Orientation (rpy in deg): roll=%.1f, pitch=%.1f, yaw=%.1f", label.c_str(), rpy.x(), rpy.y(), rpy.z());
    }

    // Callbacks
    void left_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (msg->axes.size() > 3) {
            left_grip_value_ = msg->axes[3];
        }
    }

    void left_tcp_pose_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        double mm_to_m = 0.001;
        if (msg->position.size() >= 7) {
            cur_tcp_position_ = {
                msg->position[0] * mm_to_m,
                msg->position[1] * mm_to_m,
                msg->position[2] * mm_to_m
            };
            Eigen::Quaterniond q(msg->position[3], msg->position[4], msg->position[5], msg->position[6]);  // w x y z
            cur_tcp_rotm_ = q.toRotationMatrix();
            tcp_ready_ = true;
            RCLCPP_INFO(this->get_logger(), "TCP position x,y,z = %.3f, %.3f, %.3f", cur_tcp_position_[0], cur_tcp_position_[1], cur_tcp_position_[2]);
            Eigen::Vector3d rpy = cur_tcp_rotm_.eulerAngles(0, 1, 2) * 180 / M_PI;
            RCLCPP_INFO(this->get_logger(), "TCP rpy (deg) = %.1f, %.1f, %.1f", rpy.x(), rpy.y(), rpy.z());
        }
    }

    void vr_left_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        double grip = left_grip_value_;
        bool pressed = grip > grip_engage_;
        bool released = grip < grip_release_;

        if (!tcp_ready_) {
            RCLCPP_WARN(this->get_logger(), "TCP pose not ready, ignore clutch");
            return;
        }

        // Convert msg to T_vr_now_base
        Eigen::Affine3d T_vr_now_base = Eigen::Affine3d::Identity();
        Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
        T_vr_now_base.linear() = q.toRotationMatrix();
        T_vr_now_base.translation() = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        print_pose(T_vr_now_base, "T_vr_now_base before alignment");

        // Pico → Human gesture alignment
        T_vr_now_base = T_vr_human_align_ * T_vr_now_base;
        print_pose(T_vr_now_base, "T_vr_now_base after alignment");

        std::string state = left_clutch_state_;

        // State machine
        if (state == "IDLE") {
            if (pressed) {
                left_clutch_state_ = "ENGAGING";
            }
            return;
        }

        if (state == "ENGAGING") {
            // Freeze VR start
            T_vr_start_base_ = T_vr_now_base;

            // Freeze arm start (forward_lt)
            Eigen::Affine3d T_lt_start = Eigen::Affine3d::Identity();
            T_lt_start.linear() = cur_tcp_rotm_;
            T_lt_start.translation() = Eigen::Vector3d(cur_tcp_position_[0], cur_tcp_position_[1], cur_tcp_position_[2]);
            print_pose(T_lt_start, "T_lt_start");
            T_tcp_start_forward_ = T_lt_start * T_lt_forward_;
            print_pose(*T_tcp_start_forward_, "T_tcp_start_forward");
            left_clutch_state_ = "TRACKING";
            RCLCPP_INFO(this->get_logger(), "---[LEFT] Tracking started");

            // Initialize last_rotation
            // last_rotation_ = Eigen::Quaterniond(cur_tcp_rotm_);
            Eigen::Quaterniond q(cur_tcp_rotm_);
            q.normalize();
            last_rotation_ = q;
            print_pose(*T_tcp_start_forward_, "T_tcp_start_forward for last_rotation");

            return;
        }

        if (state == "TRACKING") {
            if (released) {
                left_clutch_state_ = "IDLE";
                RCLCPP_INFO(this->get_logger(), "[LEFT] Clutch RELEASED");
                return;
            }
        }

        if (left_clutch_state_ != "TRACKING" || !T_vr_start_base_ || !T_tcp_start_forward_) {
            return;
        }

        // Incremental calculation
        Eigen::Affine3d T_vr_now_left = invert_transform(T_base_base_left_) * T_vr_now_base;
        print_pose(T_vr_now_left, "T_vr_now_left");

        Eigen::Affine3d T_vr_start_left = invert_transform(T_base_base_left_) * *T_vr_start_base_;
        print_pose(T_vr_start_left, "T_vr_start_left");

        Eigen::Affine3d T_vr_now_forward = T_vr_now_left * invert_transform(T_lt_forward_);
        print_pose(T_vr_now_forward, "T_vr_now_forward");
        Eigen::Affine3d T_vr_start_forward = T_vr_start_left * invert_transform(T_lt_forward_);
        print_pose(T_vr_start_forward, "T_vr_start_forward");

        // Delta calculation
        Eigen::Affine3d delta_T = T_vr_now_forward * invert_transform(T_vr_start_forward);
        print_pose(delta_T, "delta_T");
        Eigen::Vector3d delta_pos = delta_T.translation();
        // Eigen::AngleAxisd delta_aa(delta_T.linear());
        // if (delta_aa.angle() > M_PI) delta_aa.angle() -= 2 * M_PI;        
        // Eigen::Vector3d delta_rotvec = delta_aa.angle() * delta_aa.axis();
        Eigen::AngleAxisd delta_aa(delta_T.linear());
        Eigen::Vector3d delta_rotvec = delta_aa.angle() * delta_aa.axis();
        RCLCPP_INFO(this->get_logger(), "delta_pos=%.3f,%.3f,%.3f, delta_rot(rad)=%.3f,%.3f,%.3f, norm=%.3f",
                    delta_pos.x(), delta_pos.y(), delta_pos.z(), delta_rotvec.x(), delta_rotvec.y(), delta_rotvec.z(), delta_rotvec.norm());

        if (delta_pos.norm() > 0.5 || delta_rotvec.norm() > 1.0) {
            RCLCPP_WARN(this->get_logger(), "Delta too large! Possible VR jitter or Euler jump");
        }

        // Apply to arm start
        Eigen::Affine3d T_forward_target = delta_T * *T_tcp_start_forward_;
        print_pose(T_forward_target, "T_forward_target");
        Eigen::Affine3d T_lt_target = T_forward_target * invert_transform(T_lt_forward_);
        print_pose(T_lt_target, "T_lt_target");

        // Position
        Eigen::Vector3d pos = T_lt_target.translation();

        // Rotation
        Eigen::Matrix3d R_target = T_lt_target.linear();
        Eigen::Quaterniond Q_target(R_target);

        if (!last_rotation_) {
            Eigen::Vector3d rpy = R_target.eulerAngles(0, 1, 2);
            last_rotation_ = Q_target;
        } else {
            Eigen::Quaterniond last_q = *last_rotation_;
            Eigen::AngleAxisd last_aa(last_q);
            Eigen::Vector3d last_rotvec = last_aa.angle() * last_aa.axis();
            Eigen::Vector3d new_rotvec = last_rotvec + delta_rotvec;
            Eigen::Quaterniond new_q;
            double angle = new_rotvec.norm();
            if (angle < 1e-8) {
                new_q = Eigen::Quaterniond::Identity();
            } else {
                new_q = Eigen::Quaterniond(Eigen::AngleAxisd(angle, new_rotvec / angle));
            }
            // Eigen::Vector3d rpy = new_q.toRotationMatrix().eulerAngles(0, 1, 2);
            last_rotation_ = new_q;

            // if (delta_rotvec.norm() < 0.01) {
            //     rpy = cur_tcp_rotm_.eulerAngles(0, 1, 2);
            //     RCLCPP_INFO(this->get_logger(), "Delta small, using current TCP rpy for test");
            // }
        }

        Eigen::Vector3d rpy = (*last_rotation_).toRotationMatrix().eulerAngles(0, 1, 2);

        // Output (convert to mm)
        double meter_to_mm = 1000.0;
        std::vector<double> out_pose = {
            pos.x() * meter_to_mm,
            pos.y() * meter_to_mm,
            pos.z() * meter_to_mm,
            rpy.x(),
            rpy.y(),
            rpy.z()
        };

        RCLCPP_INFO(this->get_logger(),
                    "target_pos(mm)=(%.1f,%.1f,%.1f) target_rot(rad)=(%.3f,%.3f,%.3f) target_rot(deg)=(%.1f,%.1f,%.1f)",
                    out_pose[0], out_pose[1], out_pose[2], out_pose[3], out_pose[4], out_pose[5],
                    rpy.x() * 180 / M_PI, rpy.y() * 180 / M_PI, rpy.z() * 180 / M_PI);

        publish_servo_p_command(out_pose);
    }

    void publish_servo_p_command(const std::vector<double>& pose) {
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = this->get_clock()->now();
        msg.position = pose;
        left_servo_pub_->publish(msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QyhTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
