#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <cstring>
#include <iostream>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using namespace std::chrono_literals;

// Data packet structure matching the PICO 4 sender
#pragma pack(push, 1)
struct ControllerDataPacket {
    int64_t timestamp;           // 8 bytes
    
    // Head Pose
    float head_position[3];      // 12 bytes
    float head_orientation[4];   // 16 bytes

    // Left controller
    uint8_t left_active;         // 1 byte
    float left_position[3];      // 12 bytes: x, y, z
    float left_orientation[4];   // 16 bytes: x, y, z, w (quaternion)
    float left_joystick[2];      // 8 bytes: x, y
    float left_trigger;          // 4 bytes
    float left_grip;             // 4 bytes
    
    // Right controller
    uint8_t right_active;        // 1 byte
    float right_position[3];     // 12 bytes
    float right_orientation[4];  // 16 bytes
    float right_joystick[2];     // 8 bytes
    float right_trigger;         // 4 bytes
    float right_grip;            // 4 bytes
    
    // Buttons and touches
    uint32_t buttons_bitmask;    // 4 bytes
    uint32_t touches_bitmask;    // 4 bytes
};
#pragma pack(pop)

class VRBridgeNode : public rclcpp::Node
{
public:
    VRBridgeNode() : Node("vr_bridge_node")
    {
        // Parameters
        this->declare_parameter("udp_port", 9999);
        this->declare_parameter("frame_id", "vr_origin");
        
        udp_port_ = this->get_parameter("udp_port").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Publishers
        head_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/head/pose", 10);
        
        left_hand_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/left_hand/pose", 10);
        left_hand_joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/vr/left_hand/joy", 10);
        left_hand_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vr/left_hand/active", 10);
        
        right_hand_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vr/right_hand/pose", 10);
        right_hand_joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/vr/right_hand/joy", 10);
        right_hand_active_pub_ = this->create_publisher<std_msgs::msg::Bool>("/vr/right_hand/active", 10);

        // TF Broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Initialize UDP
        if (!init_udp()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialize UDP socket");
            return;
        }

        // Start receive thread
        running_ = true;
        recv_thread_ = std::thread(&VRBridgeNode::receive_loop, this);
        
        RCLCPP_INFO(this->get_logger(), "VR Bridge Node started, listening on port %d", udp_port_);
    }

    ~VRBridgeNode()
    {
        running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        close_udp();
    }

private:
    bool init_udp()
    {
#ifdef _WIN32
        WSADATA wsaData;
        if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
            return false;
        }
#endif
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            return false;
        }

        // Allow reuse address
        int reuse = 1;
        if (setsockopt(udp_socket_, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0) {
            RCLCPP_WARN(this->get_logger(), "Failed to set SO_REUSEADDR");
        }

        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(udp_port_);
        addr.sin_addr.s_addr = htonl(INADDR_ANY);

        if (bind(udp_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            return false;
        }

        return true;
    }

    void close_udp()
    {
        if (udp_socket_ >= 0) {
#ifdef _WIN32
            closesocket(udp_socket_);
            WSACleanup();
#else
            close(udp_socket_);
#endif
            udp_socket_ = -1;
        }
    }

    void receive_loop()
    {
        ControllerDataPacket packet;
        struct sockaddr_in client_addr;
        socklen_t addr_len = sizeof(client_addr);
        
        while (running_ && rclcpp::ok()) {
            // Use select for timeout to allow clean shutdown
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(udp_socket_, &readfds);
            
            struct timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 100000; // 100ms timeout

            int ret = select(udp_socket_ + 1, &readfds, NULL, NULL, &tv);
            
            if (ret > 0) {
                int len = recvfrom(udp_socket_, (char*)&packet, sizeof(packet), 0, (struct sockaddr*)&client_addr, &addr_len);
                
                if (len == sizeof(ControllerDataPacket)) {
                    process_packet(packet);
                } else if (len > 0) {
                    RCLCPP_WARN(this->get_logger(), "Received packet of unexpected size: %d (expected %zu)", len, sizeof(ControllerDataPacket));
                }
            }
        }
    }

    void process_packet(const ControllerDataPacket& packet)
    {
        auto now = this->now();

        // --- Head ---
        geometry_msgs::msg::PoseStamped head_msg;
        head_msg.header.stamp = now;
        head_msg.header.frame_id = frame_id_;
        head_msg.pose.position.x = packet.head_position[0];
        head_msg.pose.position.y = packet.head_position[1];
        head_msg.pose.position.z = packet.head_position[2];
        head_msg.pose.orientation.x = packet.head_orientation[0];
        head_msg.pose.orientation.y = packet.head_orientation[1];
        head_msg.pose.orientation.z = packet.head_orientation[2];
        head_msg.pose.orientation.w = packet.head_orientation[3];
        head_pose_pub_->publish(head_msg);
        broadcast_tf("vr_head", head_msg.pose, now);

        // --- Left Hand ---
        std_msgs::msg::Bool left_active_msg;
        left_active_msg.data = (packet.left_active != 0);
        left_hand_active_pub_->publish(left_active_msg);

        if (packet.left_active) {
            geometry_msgs::msg::PoseStamped left_msg;
            left_msg.header.stamp = now;
            left_msg.header.frame_id = frame_id_;
            left_msg.pose.position.x = packet.left_position[0];
            left_msg.pose.position.y = packet.left_position[1];
            left_msg.pose.position.z = packet.left_position[2];
            left_msg.pose.orientation.x = packet.left_orientation[0];
            left_msg.pose.orientation.y = packet.left_orientation[1];
            left_msg.pose.orientation.z = packet.left_orientation[2];
            left_msg.pose.orientation.w = packet.left_orientation[3];
            left_hand_pose_pub_->publish(left_msg);
            broadcast_tf("vr_left_hand", left_msg.pose, now);

            sensor_msgs::msg::Joy left_joy;
            left_joy.header.stamp = now;
            left_joy.axes.push_back(packet.left_joystick[0]); // 0: Joy X
            left_joy.axes.push_back(packet.left_joystick[1]); // 1: Joy Y
            left_joy.axes.push_back(packet.left_trigger);     // 2: Trigger
            left_joy.axes.push_back(packet.left_grip);        // 3: Grip
            // Buttons mapping (simplified)
            left_joy.buttons.push_back((packet.buttons_bitmask & (1 << 2)) ? 1 : 0); // 0: X
            left_joy.buttons.push_back((packet.buttons_bitmask & (1 << 3)) ? 1 : 0); // 1: Y
            left_joy.buttons.push_back((packet.buttons_bitmask & (1 << 4)) ? 1 : 0); // 2: Menu
            left_joy.buttons.push_back((packet.buttons_bitmask & (1 << 10)) ? 1 : 0); // 3: Joy Click
            left_hand_joy_pub_->publish(left_joy);
        }

        // --- Right Hand ---
        std_msgs::msg::Bool right_active_msg;
        right_active_msg.data = (packet.right_active != 0);
        right_hand_active_pub_->publish(right_active_msg);

        if (packet.right_active) {
            geometry_msgs::msg::PoseStamped right_msg;
            right_msg.header.stamp = now;
            right_msg.header.frame_id = frame_id_;
            right_msg.pose.position.x = packet.right_position[0];
            right_msg.pose.position.y = packet.right_position[1];
            right_msg.pose.position.z = packet.right_position[2];
            right_msg.pose.orientation.x = packet.right_orientation[0];
            right_msg.pose.orientation.y = packet.right_orientation[1];
            right_msg.pose.orientation.z = packet.right_orientation[2];
            right_msg.pose.orientation.w = packet.right_orientation[3];
            right_hand_pose_pub_->publish(right_msg);
            broadcast_tf("vr_right_hand", right_msg.pose, now);

            sensor_msgs::msg::Joy right_joy;
            right_joy.header.stamp = now;
            right_joy.axes.push_back(packet.right_joystick[0]); // 0: Joy X
            right_joy.axes.push_back(packet.right_joystick[1]); // 1: Joy Y
            right_joy.axes.push_back(packet.right_trigger);     // 2: Trigger
            right_joy.axes.push_back(packet.right_grip);        // 3: Grip
            // Buttons mapping
            right_joy.buttons.push_back((packet.buttons_bitmask & (1 << 0)) ? 1 : 0); // 0: A
            right_joy.buttons.push_back((packet.buttons_bitmask & (1 << 1)) ? 1 : 0); // 1: B
            right_joy.buttons.push_back((packet.buttons_bitmask & (1 << 5)) ? 1 : 0); // 2: Home
            right_joy.buttons.push_back((packet.buttons_bitmask & (1 << 11)) ? 1 : 0); // 3: Joy Click
            right_hand_joy_pub_->publish(right_joy);
        }
    }

    void broadcast_tf(const std::string& child_frame, const geometry_msgs::msg::Pose& pose, const rclcpp::Time& time)
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = time;
        t.header.frame_id = frame_id_;
        t.child_frame_id = child_frame;
        t.transform.translation.x = pose.position.x;
        t.transform.translation.y = pose.position.y;
        t.transform.translation.z = pose.position.z;
        t.transform.rotation = pose.orientation;
        tf_broadcaster_->sendTransform(t);
    }

    int udp_socket_ = -1;
    int udp_port_;
    std::string frame_id_;
    std::thread recv_thread_;
    std::atomic<bool> running_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr head_pose_pub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_hand_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr left_hand_joy_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr left_hand_active_pub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_hand_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr right_hand_joy_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr right_hand_active_pub_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VRBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
