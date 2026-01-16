#include "qyh_head_motor_control/bus_servo_protocol.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <iostream>
#include <iomanip>

namespace qyh_head_motor_control
{

// 调试开关 - 设为 true 打印所有收发数据
static bool g_debug_enabled = false;

// 辅助函数：打印十六进制数据
static void printHex(const char* prefix, const uint8_t* data, size_t len)
{
    if (!g_debug_enabled) return;
    std::cerr << prefix << " (" << len << " bytes): ";
    for (size_t i = 0; i < len; i++) {
        std::cerr << std::hex << std::setfill('0') << std::setw(2) << (int)data[i] << " ";
    }
    std::cerr << std::dec << std::endl;
}

// CRC8-MAXIM 计算 (polynomial 0x31反转=0x8C, init 0x00, RefIn=True, RefOut=True)
uint8_t crc8_maxim(const uint8_t* data, size_t len)
{
    uint8_t crc = 0x00;
    uint8_t polynomial = 0x8C;
    
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ polynomial;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

BusServoProtocol::BusServoProtocol(const std::string& port, int baudrate)
    : port_(port), baudrate_(baudrate), fd_(-1)
{
}

BusServoProtocol::~BusServoProtocol()
{
    close();
}

bool BusServoProtocol::open()
{
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        std::cerr << "Failed to open serial port: " << port_ << std::endl;
        return false;
    }
    
    tcgetattr(fd_, &old_tio_);
    
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    
    speed_t speed;
    switch (baudrate_) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 500000: speed = B500000; break;
        case 576000: speed = B576000; break;
        case 921600: speed = B921600; break;
        case 1000000: speed = B1000000; break;
        case 1152000: speed = B1152000; break;
        case 1500000: speed = B1500000; break;
        case 2000000: speed = B2000000; break;
        default:
            std::cerr << "Unsupported baudrate: " << baudrate_ << std::endl;
            ::close(fd_);
            fd_ = -1;
            return false;
    }
    
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    tio.c_oflag &= ~OPOST;
    
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;
    
    tcflush(fd_, TCIOFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::cerr << "Failed to set serial port attributes" << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    
    return true;
}

void BusServoProtocol::close()
{
    if (fd_ >= 0) {
        tcsetattr(fd_, TCSANOW, &old_tio_);
        ::close(fd_);
        fd_ = -1;
    }
}

std::vector<uint8_t> BusServoProtocol::buildFrame(uint8_t func, const std::vector<uint8_t>& data)
{
    std::vector<uint8_t> frame;
    frame.push_back(0xAA);
    frame.push_back(0x55);
    frame.push_back(func);
    frame.push_back(static_cast<uint8_t>(data.size()));
    frame.insert(frame.end(), data.begin(), data.end());
    
    uint8_t crc = crc8_maxim(&frame[2], frame.size() - 2);
    frame.push_back(crc);
    
    return frame;
}

bool BusServoProtocol::sendFrame(uint8_t func, const std::vector<uint8_t>& data)
{
    if (fd_ < 0) {
        return false;
    }
    
    std::vector<uint8_t> frame = buildFrame(func, data);
    
    // 打印发送的数据
    printHex("[TX]", frame.data(), frame.size());
    
    ssize_t written = write(fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
        std::cerr << "Failed to write to serial port" << std::endl;
        return false;
    }
    
    tcdrain(fd_);
    return true;
}

bool BusServoProtocol::recvFrame(std::vector<uint8_t>& data, int timeout_ms)
{
    if (fd_ < 0) {
        return false;
    }
    
    fd_set readfds;
    struct timeval tv;
    
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);
    
    int ret = select(fd_ + 1, &readfds, nullptr, nullptr, &tv);
    if (ret <= 0) {
        if (g_debug_enabled) {
            std::cerr << "[RX] select timeout or error, ret=" << ret << std::endl;
        }
        return false;
    }
    
    uint8_t header[4];
    ssize_t n = read(fd_, header, 4);
    
    if (n > 0) {
        printHex("[RX header]", header, n);
    }
    
    if (n != 4 || header[0] != 0xAA || header[1] != 0x55) {
        if (n > 0) {
            std::cerr << "[RX] Invalid header, flushing..." << std::endl;
            tcflush(fd_, TCIFLUSH);
        }
        return false;
    }
    
    uint8_t func = header[2];
    uint8_t len = header[3];
    
    std::vector<uint8_t> buffer(len + 1);
    n = read(fd_, buffer.data(), len + 1);
    
    if (n > 0) {
        printHex("[RX data+crc]", buffer.data(), n);
    }
    
    if (n != static_cast<ssize_t>(len + 1)) {
        std::cerr << "[RX] Data length mismatch, expected " << (len+1) << ", got " << n << std::endl;
        tcflush(fd_, TCIFLUSH);
        return false;
    }
    
    std::vector<uint8_t> crc_data;
    crc_data.push_back(func);
    crc_data.push_back(len);
    crc_data.insert(crc_data.end(), buffer.begin(), buffer.begin() + len);
    uint8_t calc_crc = crc8_maxim(crc_data.data(), crc_data.size());
    uint8_t recv_crc = buffer[len];
    
    if (calc_crc != recv_crc) {
        std::cerr << "[RX] CRC mismatch! calc=0x" << std::hex << (int)calc_crc 
                  << " recv=0x" << (int)recv_crc << std::dec << std::endl;
        tcflush(fd_, TCIFLUSH);
        return false;
    }
    
    data.assign(buffer.begin(), buffer.begin() + len);
    return true;
}

bool BusServoProtocol::setPosition(uint8_t servo_id, uint16_t position, uint16_t duration_ms)
{
    if (position > 1000) {
        position = 1000;
    }
    
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_POSITION);
    data.push_back(static_cast<uint8_t>(duration_ms & 0xFF));
    data.push_back(static_cast<uint8_t>((duration_ms >> 8) & 0xFF));
    data.push_back(1);
    data.push_back(servo_id);
    data.push_back(static_cast<uint8_t>(position & 0xFF));
    data.push_back(static_cast<uint8_t>((position >> 8) & 0xFF));
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::setPositions(const std::vector<uint8_t>& ids, 
                                     const std::vector<uint16_t>& positions,
                                     uint16_t duration_ms)
{
    if (ids.size() != positions.size() || ids.empty()) {
        return false;
    }
    
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_POSITION);
    data.push_back(static_cast<uint8_t>(duration_ms & 0xFF));
    data.push_back(static_cast<uint8_t>((duration_ms >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(ids.size()));
    
    for (size_t i = 0; i < ids.size(); i++) {
        uint16_t pos = positions[i];
        if (pos > 1000) {
            pos = 1000;
        }
        data.push_back(ids[i]);
        data.push_back(static_cast<uint8_t>(pos & 0xFF));
        data.push_back(static_cast<uint8_t>((pos >> 8) & 0xFF));
    }
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::readPosition(uint8_t servo_id, uint16_t& position)
{
    if (g_debug_enabled) {
        std::cerr << "\n=== readPosition(id=" << (int)servo_id << ") ===" << std::endl;
    }
    
    // 清空缓冲区
    tcflush(fd_, TCIOFLUSH);
    
    std::vector<uint8_t> data;
    data.push_back(CMD_READ_POSITION);  // 0x05
    data.push_back(servo_id);
    
    if (!sendFrame(FUNC_BUS_SERVO, data)) {
        return false;
    }
    
    // 等待舵机响应
    usleep(5000);  // 5ms
    
    // 读取所有可用数据
    uint8_t raw_buf[64];
    ssize_t total_read = 0;
    
    fd_set readfds;
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;  // 100ms
    
    FD_ZERO(&readfds);
    FD_SET(fd_, &readfds);
    
    if (select(fd_ + 1, &readfds, nullptr, nullptr, &tv) > 0) {
        total_read = read(fd_, raw_buf, sizeof(raw_buf));
    }
    
    if (total_read <= 0) {
        if (g_debug_enabled) {
            std::cerr << "[RX] No data received!" << std::endl;
        }
        return false;
    }
    
    // 打印收到的原始数据
    printHex("[RX raw]", raw_buf, total_read);
    
    // 在接收的数据中查找有效帧
    for (ssize_t i = 0; i <= total_read - 5; i++) {
        if (raw_buf[i] == 0xAA && raw_buf[i+1] == 0x55) {
            uint8_t func = raw_buf[i+2];
            uint8_t len = raw_buf[i+3];
            
            if (g_debug_enabled) {
                std::cerr << "[RX] Found frame at offset " << i 
                          << ", func=0x" << std::hex << (int)func 
                          << ", len=" << std::dec << (int)len << std::endl;
            }
            
            // 检查是否有足够的数据
            if (i + 4 + len + 1 > total_read) {
                std::cerr << "[RX] Incomplete frame" << std::endl;
                continue;
            }
            
            // 验证CRC
            std::vector<uint8_t> crc_data;
            crc_data.push_back(func);
            crc_data.push_back(len);
            for (uint8_t j = 0; j < len; j++) {
                crc_data.push_back(raw_buf[i + 4 + j]);
            }
            uint8_t calc_crc = crc8_maxim(crc_data.data(), crc_data.size());
            uint8_t recv_crc = raw_buf[i + 4 + len];
            
            if (g_debug_enabled) {
                std::cerr << "[RX] CRC: calc=0x" << std::hex << (int)calc_crc 
                          << " recv=0x" << (int)recv_crc << std::dec << std::endl;
            }
            
            if (calc_crc != recv_crc) {
                std::cerr << "[RX] CRC mismatch, skipping frame" << std::endl;
                continue;
            }
            
            // 打印帧数据内容
            printHex("[RX frame data]", &raw_buf[i + 4], len);
            
            // 解析响应
            // 回包结构: [0:ID, 1:SubCmd(0x05), 2:Status, 3:PosLo, 4:PosHi]
            uint8_t* resp = &raw_buf[i + 4];
            
            if (g_debug_enabled) {
                std::cerr << "[RX] resp[0]=0x" << std::hex << (int)resp[0]
                          << " resp[1]=0x" << (int)resp[1]
                          << " resp[2]=0x" << (int)resp[2] << std::dec << std::endl;
            }
            
            if (len >= 5 && resp[1] == CMD_READ_POSITION) {
                if (resp[0] != servo_id) {
                    std::cerr << "[RX] ID mismatch: expected " << (int)servo_id 
                              << ", got " << (int)resp[0] << std::endl;
                }
                
                int8_t status = static_cast<int8_t>(resp[2]);
                if (status != 0) {
                    std::cerr << "[RX] Servo error status: " << (int)status << std::endl;
                    return false;
                }
                
                position = resp[3] | (resp[4] << 8);
                if (g_debug_enabled) {
                    std::cerr << "[RX] Position: " << position << std::endl;
                }
                return true;
            }
        }
    }
    
    std::cerr << "[RX] No valid frame found" << std::endl;
    return false;
}

bool BusServoProtocol::setServoId(uint8_t old_id, uint8_t new_id)
{
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_ID);
    data.push_back(old_id);
    data.push_back(new_id);
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::readServoId(uint8_t& id)
{
    tcflush(fd_, TCIOFLUSH);
    
    std::vector<uint8_t> data;
    data.push_back(CMD_READ_ID);
    data.push_back(0xFE);
    
    if (!sendFrame(FUNC_BUS_SERVO, data)) {
        return false;
    }
    
    usleep(5000);
    
    std::vector<uint8_t> response;
    if (!recvFrame(response, 100)) {
        return false;
    }
    
    if (response.size() >= 4 && response[1] == CMD_READ_ID) {
        id = response[3];
        return true;
    }
    
    return false;
}

bool BusServoProtocol::enableTorque(uint8_t servo_id, bool enable)
{
    std::vector<uint8_t> data;
    data.push_back(enable ? CMD_DISABLE_TORQUE : CMD_ENABLE_TORQUE);
    data.push_back(servo_id);
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::setAngleLimit(uint8_t servo_id, uint16_t min_pos, uint16_t max_pos)
{
    if (min_pos > 1000) min_pos = 1000;
    if (max_pos > 1000) max_pos = 1000;
    
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_ANGLE_LIMIT);
    data.push_back(servo_id);
    data.push_back(static_cast<uint8_t>(min_pos & 0xFF));
    data.push_back(static_cast<uint8_t>((min_pos >> 8) & 0xFF));
    data.push_back(static_cast<uint8_t>(max_pos & 0xFF));
    data.push_back(static_cast<uint8_t>((max_pos >> 8) & 0xFF));
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::setOffset(uint8_t servo_id, int8_t offset)
{
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_OFFSET);
    data.push_back(servo_id);
    data.push_back(static_cast<uint8_t>(offset));
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

bool BusServoProtocol::saveOffset(uint8_t servo_id)
{
    std::vector<uint8_t> data;
    data.push_back(CMD_SAVE_OFFSET);
    data.push_back(servo_id);
    
    return sendFrame(FUNC_BUS_SERVO, data);
}

}  // namespace qyh_head_motor_control
