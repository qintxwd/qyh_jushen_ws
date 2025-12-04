/*
 * Copyright 2024 - 2024 PICO. All rights reserved.
 *
 * NOTICE: All information contained herein is, and remains the property of PICO.
 * The intellectual and technical concepts contained herein are proprietary to PICO.
 * and may be covered by patents, patents in process, and are protected by trade
 * secret or copyright law. Dissemination of this information or reproduction of
 * this material is strictly forbidden unless prior written permission is obtained
 * from PICO.
 */

#include "AndroidOpenXrProgram.h"
#include "CheckUtils.h"
#include "Cube.h"
#include "imgui_internal.h"
#include "GuiWindow.h"
#include "GuiPlane.h"
#include "CartesianBranch.h"
#include <sstream>
#include <iomanip>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <errno.h>

using namespace PVRSampleFW;

// UDP broadcast configuration
const int UDP_PORT = 9999;

// Data packet structure for UDP broadcast
// 使用 #pragma pack 确保结构体紧凑排列，避免对齐问题
#pragma pack(push, 1)
struct ControllerDataPacket {
    int64_t timestamp;           // 8 bytes
    
    // Head Pose
    float head_position[3];      // 12 bytes
    float head_orientation[4];   // 16 bytes

    // Left controller
    uint8_t left_active;         // 1 byte (用 uint8_t 替代 bool)
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
// Total: 8 + 12 + 16 + 1 + 12 + 16 + 8 + 4 + 4 + 1 + 12 + 16 + 8 + 4 + 4 + 4 + 4 = 134 bytes

class QyhTest : public AndroidOpenXrProgram {
public:
    explicit QyhTest(const std::shared_ptr<PVRSampleFW::Configurations> &appConfigParam)
        : AndroidOpenXrProgram(appConfigParam) {
    }

    QyhTest() : AndroidOpenXrProgram() {
    }

    virtual ~QyhTest() {
        if (udp_socket_ >= 0) {
            close(udp_socket_);
            udp_socket_ = -1;
        }
    }

    bool CustomizedAppPostInit() override {
        AndroidOpenXrProgram::CustomizedAppPostInit();
        InitUdpSocket();
        AddControllerVisuals();
        AddGuiPlane();
        
        // 打印结构体大小用于调试
        PLOGI("ControllerDataPacket size: %zu bytes", sizeof(ControllerDataPacket));
        return true;
    }

    bool CustomizedXrInputHandlerSetup() override {
        AndroidOpenXrProgram::CustomizedXrInputHandlerSetup();

        // register the input callback to the openxr wrapper.
        auto handleInputFunc = [](class BasicOpenXrWrapper *openxr, const PVRSampleFW::XrFrameIn &frameIn) {
            auto pOpenXrAppWrapper = dynamic_cast<QyhTest *>(openxr);
            pOpenXrAppWrapper->UpdateControllerData(frameIn);
        };
        RegisterHandleInputFunc(handleInputFunc);
        return true;
    }

    bool CustomizedPreRenderFrame() override {
        UpdateControllers();
        UpdateGuiText();
        BroadcastControllerData();
        return true;
    }

    /// Override the CustomizedRender function render what you want besides the scene paradigm
    bool CustomizedRender() override {
        return true;
    }

    /// Override the CustomizedExtensionAndFeaturesInit function to register features those you want to activate
    void CustomizedExtensionAndFeaturesInit() override {
        AndroidOpenXrProgram::CustomizedExtensionAndFeaturesInit();
    }

    std::string GetApplicationName() override {
        return "QyhTest";
    }

private:
    void InitUdpSocket() {
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            PLOGE("Failed to create UDP socket: %s", strerror(errno));
            return;
        }

        // Enable broadcast
        int broadcast_enable = 1;
        if (setsockopt(udp_socket_, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0) {
            PLOGE("Failed to enable broadcast: %s", strerror(errno));
            close(udp_socket_);
            udp_socket_ = -1;
            return;
        }

        memset(&broadcast_addr_, 0, sizeof(broadcast_addr_));
        broadcast_addr_.sin_family = AF_INET;
        broadcast_addr_.sin_port = htons(UDP_PORT);
        broadcast_addr_.sin_addr.s_addr = inet_addr("255.255.255.255");

        PLOGI("UDP socket initialized, broadcasting to port %d", UDP_PORT);
        PLOGI("Packet size: %zu bytes", sizeof(ControllerDataPacket));
    }

    void BroadcastControllerData() {
        if (udp_socket_ < 0) {
            return;
        }

        // Prepare data packet
        ControllerDataPacket packet;
        memset(&packet, 0, sizeof(packet));  // 初始化为0
        
        packet.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        // Head Pose
        packet.head_position[0] = head_pose_.position.x;
        packet.head_position[1] = head_pose_.position.y;
        packet.head_position[2] = head_pose_.position.z;
        packet.head_orientation[0] = head_pose_.orientation.x;
        packet.head_orientation[1] = head_pose_.orientation.y;
        packet.head_orientation[2] = head_pose_.orientation.z;
        packet.head_orientation[3] = head_pose_.orientation.w;

        // Left controller
        packet.left_active = controller_active_[Side::LEFT] ? 1 : 0;
        if (packet.left_active) {
            packet.left_position[0] = controller_pose_[Side::LEFT].position.x;
            packet.left_position[1] = controller_pose_[Side::LEFT].position.y;
            packet.left_position[2] = controller_pose_[Side::LEFT].position.z;
            packet.left_orientation[0] = controller_pose_[Side::LEFT].orientation.x;
            packet.left_orientation[1] = controller_pose_[Side::LEFT].orientation.y;
            packet.left_orientation[2] = controller_pose_[Side::LEFT].orientation.z;
            packet.left_orientation[3] = controller_pose_[Side::LEFT].orientation.w;
            packet.left_joystick[0] = left_joystick_.x;
            packet.left_joystick[1] = left_joystick_.y;
            packet.left_trigger = trigger_value_[Side::LEFT];
            packet.left_grip = grip_value_[Side::LEFT];
        }

        // Right controller
        packet.right_active = controller_active_[Side::RIGHT] ? 1 : 0;
        if (packet.right_active) {
            packet.right_position[0] = controller_pose_[Side::RIGHT].position.x;
            packet.right_position[1] = controller_pose_[Side::RIGHT].position.y;
            packet.right_position[2] = controller_pose_[Side::RIGHT].position.z;
            packet.right_orientation[0] = controller_pose_[Side::RIGHT].orientation.x;
            packet.right_orientation[1] = controller_pose_[Side::RIGHT].orientation.y;
            packet.right_orientation[2] = controller_pose_[Side::RIGHT].orientation.z;
            packet.right_orientation[3] = controller_pose_[Side::RIGHT].orientation.w;
            packet.right_joystick[0] = right_joystick_.x;
            packet.right_joystick[1] = right_joystick_.y;
            packet.right_trigger = trigger_value_[Side::RIGHT];
            packet.right_grip = grip_value_[Side::RIGHT];
        }

        // Buttons and touches
        packet.buttons_bitmask = buttons_bitmask_;
        packet.touches_bitmask = touches_bitmask_;

        // Send UDP packet
        ssize_t sent = sendto(udp_socket_, &packet, sizeof(packet), 0,
                             (struct sockaddr*)&broadcast_addr_, sizeof(broadcast_addr_));
        if (sent < 0) {
            PLOGE("Failed to send UDP packet: %s", strerror(errno));
        }
    }

    void UpdateControllerData(const PVRSampleFW::XrFrameIn &frameIn) {
        // Store the latest frame data
        head_pose_ = frameIn.head_pose;
        for (int hand = 0; hand < Side::COUNT; hand++) {
            controller_active_[hand] = frameIn.controller_actives[hand];
            controller_pose_[hand] = frameIn.controller_poses[hand];
            controller_aim_pose_[hand] = frameIn.controller_aim_poses[hand];
            trigger_value_[hand] = frameIn.controller_trigger_value[hand];
            grip_value_[hand] = frameIn.controller_grip_value[hand];
        }
        left_joystick_ = frameIn.left_joystick_position;
        right_joystick_ = frameIn.right_joystick_position;
        buttons_bitmask_ = frameIn.all_buttons_bitmask;
        touches_bitmask_ = frameIn.all_touches_bitmask;
    }

    void UpdateControllers() {
        Scene &scene = scenes_.at(SAMPLE_SCENE_TYPE_CONTROLLER);
        for (int i = 0; i < Side::COUNT; i++) {
            auto hand = scene.GetObject(controller_ids_[i]);
            auto ray = scene.GetObject(aim_ids_[i]);
            if (controller_active_[i]) {
                hand->SetPose(controller_pose_[i]);
                ray->SetPose(controller_aim_pose_[i]);
            }
        }
    }

    void UpdateGuiText() {
        if (gui_window_ != nullptr) {
            // Format controller pose information
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(3);
            
            // Packet info
            oss << "UDP Port: " << UDP_PORT << " | Packet: " << sizeof(ControllerDataPacket) << " bytes\n\n";
            
            // Head Pose
            oss << "=== Head Pose ===\n";
            oss << "Position: (" 
                << head_pose_.position.x << ", "
                << head_pose_.position.y << ", "
                << head_pose_.position.z << ")\n";
            oss << "Orientation: (" 
                << head_pose_.orientation.x << ", "
                << head_pose_.orientation.y << ", "
                << head_pose_.orientation.z << ", "
                << head_pose_.orientation.w << ")\n\n";

            // Left Controller
            oss << "=== Left Controller ===\n";
            if (controller_active_[Side::LEFT]) {
                oss << "Active: YES\n";
                oss << "Position: (" 
                    << controller_pose_[Side::LEFT].position.x << ", "
                    << controller_pose_[Side::LEFT].position.y << ", "
                    << controller_pose_[Side::LEFT].position.z << ")\n";
                oss << "Orientation: (" 
                    << controller_pose_[Side::LEFT].orientation.x << ", "
                    << controller_pose_[Side::LEFT].orientation.y << ", "
                    << controller_pose_[Side::LEFT].orientation.z << ", "
                    << controller_pose_[Side::LEFT].orientation.w << ")\n";
                oss << "Joystick: (" << left_joystick_.x << ", " << left_joystick_.y << ")\n";
                oss << "Trigger: " << trigger_value_[Side::LEFT] << "\n";
                oss << "Grip: " << grip_value_[Side::LEFT] << "\n";
            } else {
                oss << "Active: NO\n";
            }
            
            oss << "\n=== Right Controller ===\n";
            if (controller_active_[Side::RIGHT]) {
                oss << "Active: YES\n";
                oss << "Position: (" 
                    << controller_pose_[Side::RIGHT].position.x << ", "
                    << controller_pose_[Side::RIGHT].position.y << ", "
                    << controller_pose_[Side::RIGHT].position.z << ")\n";
                oss << "Orientation: (" 
                    << controller_pose_[Side::RIGHT].orientation.x << ", "
                    << controller_pose_[Side::RIGHT].orientation.y << ", "
                    << controller_pose_[Side::RIGHT].orientation.z << ", "
                    << controller_pose_[Side::RIGHT].orientation.w << ")\n";
                oss << "Joystick: (" << right_joystick_.x << ", " << right_joystick_.y << ")\n";
                oss << "Trigger: " << trigger_value_[Side::RIGHT] << "\n";
                oss << "Grip: " << grip_value_[Side::RIGHT] << "\n";
            } else {
                oss << "Active: NO\n";
            }
            
            // Button states
            oss << "\n=== Buttons (Pressed) ===\n";
            if (buttons_bitmask_ & PVRSampleFW::XrFrameIn::kButtonA) oss << "A ";
            if (buttons_bitmask_ & PVRSampleFW::XrFrameIn::kButtonB) oss << "B ";
            if (buttons_bitmask_ & PVRSampleFW::XrFrameIn::kButtonX) oss << "X ";
            if (buttons_bitmask_ & PVRSampleFW::XrFrameIn::kButtonY) oss << "Y ";
            if (buttons_bitmask_ & PVRSampleFW::XrFrameIn::kButtonMenu) oss << "Menu ";
            if (buttons_bitmask_ & PVRSampleFW::XrFrameIn::kButtonHome) oss << "Home ";
            if (buttons_bitmask_ & PVRSampleFW::XrFrameIn::kButtonJoystickLeft) oss << "Joy-L ";
            if (buttons_bitmask_ & PVRSampleFW::XrFrameIn::kButtonJoystickRight) oss << "Joy-R ";
            if (buttons_bitmask_ == 0) oss << "None";
            oss << "\n";
            
            // Touch states
            oss << "\n=== Touches ===\n";
            if (touches_bitmask_ & PVRSampleFW::XrFrameIn::kTouchJoystickLeft) oss << "Joy-L ";
            if (touches_bitmask_ & PVRSampleFW::XrFrameIn::kTouchJoystickRight) oss << "Joy-R ";
            if (touches_bitmask_ & PVRSampleFW::XrFrameIn::kTouchTriggerLeft) oss << "Trigger-L ";
            if (touches_bitmask_ & PVRSampleFW::XrFrameIn::kTouchTriggerRight) oss << "Trigger-R ";
            if (touches_bitmask_ == 0) oss << "None";
            
            gui_window_->UpdateText(text_component_id_, oss.str().c_str());
        }
    }

    void AddControllerVisuals() {
        Scene &scene = scenes_.at(SAMPLE_SCENE_TYPE_CONTROLLER);
        
        // Add hand cubes for visualization
        for (int i = 0; i < Side::COUNT; i++) {
            XrPosef handPose = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f}};
            float scale = 0.05f;
            XrVector3f handScale = {scale, scale, scale};
            auto cubeHand = std::make_shared<PVRSampleFW::Cube>(handPose, handScale);
            controller_ids_[i] = scene.AddObject(cubeHand);
        }
        
        // Add aim rays
        for (int i = 0; i < Side::COUNT; i++) {
            XrPosef handPose = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.0f}};
            float scale = 0.1f;
            XrVector3f handScale = {scale, scale, scale};
            auto cartesianBranch = std::make_shared<PVRSampleFW::CartesianBranch>(handPose, handScale);
            aim_ids_[i] = scene.AddObject(cartesianBranch);
        }
    }

    void AddGuiPlane() {
        GuiWindow::Builder builder;
        gui_window_ = builder.SetTitle("Controller Status")
                .SetSize(1200, 900)
                .SetBgColor(0.1f, 0.1f, 0.2f, 0.95f)
                .SetText("Initializing...")
                .SetFontSize(18)
                .SetTextColor(0.0f, 1.0f, 0.0f, 1.0f)
                .Build();
        
        text_component_id_ = gui_window_->AddText("Waiting for data...");
        gui_window_->SetComponentBgColor(text_component_id_, 0.05f, 0.05f, 0.1f, 0.9f);
        gui_window_->SetComponentSize(text_component_id_, 1100, 800);
        gui_window_->SetComponentTextColor(text_component_id_, 0.0f, 1.0f, 0.0f, 1.0f);
        gui_window_->SetComponentTextSize(text_component_id_, 16);
        gui_window_->SetComponentPos(text_component_id_, 50, 50);

        Scene &guiScene = scenes_.at(SAMPLE_SCENE_TYPE_GUI);
        XrPosef guiPose = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, -1.5f}};
        XrVector3f guiScale = {2.0f, 1.5f, 1.0f};
        auto guiPlane = std::make_shared<GuiPlane>(guiPose, guiScale, gui_window_);
        guiScene.AddObject(guiPlane);
    }

private:
    int64_t controller_ids_[Side::COUNT] = {-1, -1};
    int64_t aim_ids_[Side::COUNT] = {-1, -1};
    
    // Controller state data
    XrBool32 controller_active_[Side::COUNT] = {XR_FALSE, XR_FALSE};
    XrPosef controller_pose_[Side::COUNT] = {};
    XrPosef controller_aim_pose_[Side::COUNT] = {};
    XrPosef head_pose_ = {};  // Head pose
    float trigger_value_[Side::COUNT] = {0.0f, 0.0f};
    float grip_value_[Side::COUNT] = {0.0f, 0.0f};
    XrVector2f left_joystick_ = {0.0f, 0.0f};
    XrVector2f right_joystick_ = {0.0f, 0.0f};
    uint32_t buttons_bitmask_ = 0;
    uint32_t touches_bitmask_ = 0;
    
    // GUI components
    std::shared_ptr<GuiWindow> gui_window_;
    int text_component_id_ = -1;
    
    // UDP broadcast
    int udp_socket_ = -1;
    struct sockaddr_in broadcast_addr_;
};

void android_main(struct android_app *app) {
    PLOGI("QyhTest android_main()");
    auto config = std::make_shared<Configurations>();
    auto program = std::make_shared<QyhTest>(config);
    program->Run(app);
}
