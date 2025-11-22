#include <iostream>
#include <modbus/modbus.h>
#include <thread>
#include <chrono>
#include <vector>
#include <algorithm> // for std::max, std::min

// 寄存器地址定义
#define REG_CONTROL     0x03E8
#define REG_STATUS      0x07D0

// 状态结构体 (增加力反馈字段)
struct GripperState {
    bool is_activated;      // 是否已激活
    int object_status;      // 物体检测状态 (0:运动中, 1:内撑抓到, 2:外夹抓到, 3:到达位置)
    int current_position;   // 实时位置 (0-255)
    int current_force;      // 实时力反馈/电流 (0-255) [新增]
    int fault_code;         // 故障码
};

class JodellGripper {
private:
    modbus_t *ctx;
    int slave_id;

public:
    JodellGripper(const char* device, int baud = 115200, int slave = 1) {
        ctx = modbus_new_rtu(device, baud, 'N', 8, 1);
        if (!ctx) {
            std::cerr << "无法创建 Modbus 上下文" << std::endl;
            exit(1);
        }
        this->slave_id = slave;
        modbus_set_slave(ctx, slave_id);
        modbus_set_response_timeout(ctx, 0, 500000); // 500ms 超时
    }

    ~JodellGripper() {
        if (ctx) {
            modbus_close(ctx);
            modbus_free(ctx);
        }
    }

    bool connect() {
        if (modbus_connect(ctx) == -1) {
            std::cerr << "连接失败: " << modbus_strerror(errno) << std::endl;
            return false;
        }
        return true;
    }

    // 激活夹爪
    bool activate() {
        // 步骤1: Reset
        modbus_write_register(ctx, REG_CONTROL, 0x0000);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // 步骤2: Enable
        if (modbus_write_register(ctx, REG_CONTROL, 0x0001) == -1) return false;

        // 步骤3: 等待激活完成
        std::cout << "正在激活..." << std::endl;
        for (int i = 0; i < 50; i++) {
            GripperState state = getStatus();
            if (state.is_activated) {
                std::cout << "激活成功!" << std::endl;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return false;
    }

    // 发送运动指令
    // force_limit: 0-255 (力矩限制，决定了最大抓取力)
    bool move(int pos, int speed, int force_limit) {
        uint16_t values[3];
        
        // 限制范围
        pos = std::max(0, std::min(255, pos));
        speed = std::max(0, std::min(255, speed));
        force_limit = std::max(0, std::min(255, force_limit));

        values[0] = 0x0009; // 0x03E8: Enable(Bit0) + Go(Bit3)
        values[1] = (uint16_t)(pos << 8); // 0x03E9: High Byte = Position
        values[2] = (uint16_t)((force_limit << 8) | speed); // 0x03EA: High=Force, Low=Speed

        return modbus_write_registers(ctx, REG_CONTROL, 3, values) != -1;
    }

    // 获取详细状态 (包含力反馈)
    GripperState getStatus() {
        GripperState state = {false, 0, 0, 0, 0};
        uint16_t read_buf[3]; // 读取 0x07D0, 0x07D1, 0x07D2

        // 使用 0x04 功能码读取输入寄存器 (Input Registers)
        // 注意: 有些设备可能使用 0x03 (Holding Registers)，如果读不到请尝试改用 modbus_read_registers
        int rc = modbus_read_input_registers(ctx, REG_STATUS, 3, read_buf);
        if (rc == -1) {
            // 尝试回退到读保持寄存器
            rc = modbus_read_registers(ctx, REG_STATUS, 3, read_buf);
            if (rc == -1) return state;
        }

        // --- 解析 0x07D0 (状态字) ---
        // Bit 0: gACT (激活状态)
        state.is_activated = (read_buf[0] >> 0) & 0x01;
        // Bit 6-7: gOBJ (物体检测状态)
        state.object_status = (read_buf[0] >> 6) & 0x03;

        // --- 解析 0x07D1 (故障与位置) ---
        state.fault_code = read_buf[1] & 0x00FF;        // 低字节: 故障码
        state.current_position = (read_buf[1] >> 8);    // 高字节: 实际位置

        // --- 解析 0x07D2 (力反馈与速度) [具身智能关键数据] ---
        // High Byte: 实时力/电流 (Force Status)
        // Low Byte: 实时速度
        state.current_force = (read_buf[2] >> 8); 

        return state;
    }
};

int main() {
    // 修改为您实际的串口设备
    JodellGripper gripper("/dev/ttyUSB0", 115200, 1);

    if (!gripper.connect()) {
        std::cerr << "连接失败!" << std::endl;
        return -1;
    }

    if (!gripper.activate()) {
        std::cerr << "激活失败!" << std::endl;
        return -1;
    }

    // 示例：柔性抓取测试
    // 设定目标: 全闭(255), 速度中等(100), 力限制较小(50) -> 模拟轻抓易碎品
    std::cout << "开始柔性抓取..." << std::endl;
    gripper.move(255, 100, 50);

    // 循环监控，直到抓到物体或动作结束
    while (true) {
        GripperState st = gripper.getStatus();

        // 打印实时数据，特别是力反馈
        std::cout << "\r位置: " << st.current_position 
                  << " | 力反馈(电流): " << st.current_force // 实时显示受力情况
                  << " | 状态: " << st.object_status 
                  << "   " << std::flush;

        // 具身智能逻辑示例：
        // 如果检测到力反馈超过某个阈值（例如20），即使 gOBJ 还没跳变，也可以认为接触到了
        if (st.current_force > 20) {
            // std::cout << " [感知到接触] "; 
        }

        // gOBJ: 2=外夹抓到, 3=到位停止
        if (st.object_status == 2) {
            std::cout << "\n检测到物体! 抓取成功。" << std::endl;
            break;
        } else if (st.object_status == 3) {
            std::cout << "\n到达位置，未检测到物体。" << std::endl;
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 松开
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "松开..." << std::endl;
    gripper.move(0, 255, 255);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    return 0;
}