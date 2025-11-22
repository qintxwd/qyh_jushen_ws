#include <iostream>
#include <modbus/modbus.h>
#include <thread>
#include <chrono>
#include <vector>
#include <iomanip>

// 寄存器地址定义 (Hex) [cite: 198]
#define REG_CONTROL     0x03E8
#define REG_POSITION    0x03E9
#define REG_SPEED_FORCE 0x03EA
#define REG_STATUS      0x07D0
#define REG_FAULT       0x07D1
#define REG_CURRENT     0x07D2

// 状态结构体
struct GripperState {
    bool is_activated;      // 是否已激活
    bool is_moving;         // 是否正在运动
    int object_status;      // 0:运动中, 1:内撑抓到, 2:外夹抓到, 3:到达位置(未抓到) 
    int current_position;   // 0-255
    int fault_code;         // 0为无故障 
};

class JodellGripper {
private:
    modbus_t *ctx;
    int slave_id;

public:
    JodellGripper(const char* device, int baud = 115200, int slave = 1) {
        // 初始化 Modbus RTU [cite: 182]
        ctx = modbus_new_rtu(device, baud, 'N', 8, 1);
        if (ctx == NULL) {
            std::cerr << "无法创建 Modbus 上下文" << std::endl;
            return;
        }
        this->slave_id = slave;
        modbus_set_slave(ctx, slave_id);
        
        // 设置超时时间 (根据需要调整)
        modbus_set_response_timeout(ctx, 0, 500000); // 500ms
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
        std::cout << "已连接到夹爪 ID: " << slave_id << std::endl;
        return true;
    }

    // 激活/初始化夹爪 [cite: 201, 225]
    bool activate() {
        uint16_t reg_val;
        int rc;

        std::cout << "正在激活夹爪..." << std::endl;

        // 第一步：写入 0x0000 清除状态 (Reset)
        reg_val = 0x0000;
        rc = modbus_write_register(ctx, REG_CONTROL, reg_val);
        if (rc == -1) return false;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // 第二步：写入 0x0001 使能 (Set rACT=1)
        reg_val = 0x0001;
        rc = modbus_write_register(ctx, REG_CONTROL, reg_val);
        if (rc == -1) return false;

        // 第三步：等待激活完成
        // 循环读取 0x07D0，检查 Bit0 (gACT) 是否为 1，且 Bit3 (gSTA) 是否表示完成
        // 根据文档，激活完成时 0x07D0 的内容通常包含 gACT=1 
        for (int i = 0; i < 50; i++) { // 尝试等待约5秒
            uint16_t status;
            modbus_read_registers(ctx, REG_STATUS, 1, &status);
            // gACT (Bit 0) == 1 表示激活中或已激活
            // gSTA (Bit 4-5) 可能在激活过程中变化，这里简单判断 gACT 是否置位
            if ((status & 0x01) == 1) { 
                std::cout << "夹爪激活成功!" << std::endl;
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cerr << "激活超时" << std::endl;
        return false;
    }

    // 控制运动 
    // pos: 0(全开) - 255(全闭)
    // speed: 0-255
    // force: 0-255
    bool move(int pos, int speed, int force) {
        uint16_t values[3];

        // 限制参数范围
        pos = std::max(0, std::min(255, pos));
        speed = std::max(0, std::min(255, speed));
        force = std::max(0, std::min(255, force));

        // 寄存器 0x03E8 (Control):
        // Bit0(rACT)=1 (保持激活)
        // Bit3(rGTO)=1 (前往目标)
        // 二进制: 0000 1001 = 0x09
        values[0] = 0x0009;

        // 寄存器 0x03E9 (Position):
        // 高字节 = Position, 低字节 = 保留(0) 
        values[1] = (uint16_t)(pos << 8);

        // 寄存器 0x03EA (Force & Speed):
        // 高字节 = Force, 低字节 = Speed [cite: 207, 209]
        values[2] = (uint16_t)((force << 8) | speed);

        // 使用功能码 0x10 连续写入 3 个寄存器
        int rc = modbus_write_registers(ctx, REG_CONTROL, 3, values);
        if (rc == -1) {
            std::cerr << "写入运动指令失败: " << modbus_strerror(errno) << std::endl;
            return false;
        }
        return true;
    }

    // 获取实时状态 [cite: 213, 217, 219]
    GripperState getStatus() {
        GripperState state = {false, false, 0, 0, 0};
        uint16_t read_buf[3]; // 读取 0x07D0, 0x07D1, 0x07D2

        int rc = modbus_read_registers(ctx, REG_STATUS, 3, read_buf);
        if (rc == -1) {
            std::cerr << "读取状态失败" << std::endl;
            return state;
        }

        // 解析 0x07D0 (Status) 
        uint16_t reg_status = read_buf[0];
        state.is_activated = (reg_status >> 0) & 0x01; // Bit 0: gACT
        
        // Bit 3: gGTO (1=Going to position/Stopped at pos, 0=Stopped/Reset)
        // Bit 6-7: gOBJ (物体检测)
        // 00: Moving, 01: Detected(Open), 10: Detected(Close), 11: Arrived
        state.object_status = (reg_status >> 6) & 0x03;
        state.is_moving = (state.object_status == 0 && ((reg_status >> 3) & 0x01));

        // 解析 0x07D1 (Fault & Position) [cite: 217, 219]
        uint16_t reg_fault_pos = read_buf[1];
        state.fault_code = reg_fault_pos & 0x00FF;     // 低字节: 故障码
        state.current_position = (reg_fault_pos >> 8); // 高字节: 实际位置

        return state;
    }
};

int main() {
    // 替换为您的实际串口端口，例如 "/dev/ttyUSB0"
    JodellGripper gripper("/dev/ttyUSB0", 115200, 1);

    if (!gripper.connect()) return -1;

    // 1. 激活夹爪 (上电后必须执行)
    if (!gripper.activate()) return -1;

    // 2. 闭合夹爪 (抓取)
    std::cout << "执行闭合动作..." << std::endl;
    // 参数: 位置255(闭), 速度255(快), 力度150(中等) [cite: 207, 209]
    gripper.move(255, 255, 150); 

    // 循环检测状态直到动作完成
    while (true) {
        GripperState status = gripper.getStatus();
        std::cout << "位置: " << status.current_position 
                  << " | 状态码: " << status.object_status 
                  << " | 故障: " << status.fault_code << "\r" << std::flush;

        // gOBJ状态 2 表示外夹检测到物体，3 表示到达位置(没抓到) 
        if (status.object_status == 2 || status.object_status == 3) {
            std::cout << std::endl << "动作结束: " 
                      << (status.object_status == 2 ? "抓到物体" : "到达终点") << std::endl;
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. 张开夹爪
    std::cout << "执行张开动作..." << std::endl;
    // 参数: 位置0(开), 速度255, 力度255
    gripper.move(0, 255, 255);

    // 简单等待
    std::this_thread::sleep_for(std::chrono::seconds(2));
    GripperState final_st = gripper.getStatus();
    std::cout << "最终位置: " << final_st.current_position << std::endl;

    return 0;
}