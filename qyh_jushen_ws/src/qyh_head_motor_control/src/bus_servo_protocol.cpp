#include "qyh_head_motor_control/bus_servo_protocol.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>
#include <cstring>
#include <iostream>

namespace qyh_head_motor_control
{

// CRC8-MAXIM 查找表 (polynomial 0x31, init 0xFF, refin true, refout true, xorout 0x00)
static const uint8_t CRC8_TABLE[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

uint8_t crc8_maxim(const uint8_t* data, size_t len)
{
    uint8_t crc = 0xFF;  // init value
    for (size_t i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    return ~crc;  // output XOR (反转)
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
    
    // 保存旧配置
    tcgetattr(fd_, &old_tio_);
    
    // 新配置
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    
    // 波特率
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
    
    // 8N1, no flow control
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    
    // Raw input
    tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    
    // Raw output
    tio.c_oflag &= ~OPOST;
    
    // Timeout settings
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;  // 100ms timeout
    
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
    frame.push_back(0xAA);  // Header 1
    frame.push_back(0x55);  // Header 2
    frame.push_back(func);  // Function code
    frame.push_back(static_cast<uint8_t>(data.size()));  // Data length
    frame.insert(frame.end(), data.begin(), data.end());  // Data
    
    // CRC (从 func 开始计算)
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
    
    ssize_t written = write(fd_, frame.data(), frame.size());
    if (written != static_cast<ssize_t>(frame.size())) {
        std::cerr << "Failed to write to serial port" << std::endl;
        return false;
    }
    
    tcdrain(fd_);  // 等待发送完成
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
        return false;  // Timeout or error
    }
    
    // 读取帧头
    uint8_t header[4];
    ssize_t n = read(fd_, header, 4);
    if (n != 4 || header[0] != 0xAA || header[1] != 0x55) {
        return false;
    }
    
    uint8_t func = header[2];
    uint8_t len = header[3];
    
    // 读取数据和CRC
    std::vector<uint8_t> buffer(len + 1);
    n = read(fd_, buffer.data(), len + 1);
    if (n != static_cast<ssize_t>(len + 1)) {
        return false;
    }
    
    // 验证CRC
    std::vector<uint8_t> crc_data;
    crc_data.push_back(func);
    crc_data.push_back(len);
    crc_data.insert(crc_data.end(), buffer.begin(), buffer.begin() + len);
    uint8_t crc = crc8_maxim(crc_data.data(), crc_data.size());
    
    if (crc != buffer[len]) {
        std::cerr << "CRC mismatch" << std::endl;
        return false;
    }
    
    data.assign(buffer.begin(), buffer.begin() + len);
    return true;
}

bool BusServoProtocol::setPosition(uint8_t servo_id, uint16_t position, uint16_t duration_ms)
{
    // 限制位置范围
    if (position > 1000) {
        position = 1000;
    }
    
    std::vector<uint8_t> data;
    data.push_back(CMD_SET_POSITION);  // 子命令
    data.push_back(1);                  // 舵机数量
    data.push_back(static_cast<uint8_t>(duration_ms & 0xFF));         // 时间低字节
    data.push_back(static_cast<uint8_t>((duration_ms >> 8) & 0xFF));  // 时间高字节
    data.push_back(servo_id);           // 舵机ID
    data.push_back(static_cast<uint8_t>(position & 0xFF));            // 位置低字节
    data.push_back(static_cast<uint8_t>((position >> 8) & 0xFF));     // 位置高字节
    
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
    data.push_back(CMD_SET_POSITION);  // 子命令
    data.push_back(static_cast<uint8_t>(ids.size()));  // 舵机数量
    data.push_back(static_cast<uint8_t>(duration_ms & 0xFF));         // 时间低字节
    data.push_back(static_cast<uint8_t>((duration_ms >> 8) & 0xFF));  // 时间高字节
    
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
    std::vector<uint8_t> data;
    data.push_back(CMD_READ_POSITION);
    data.push_back(1);  // 数量
    data.push_back(servo_id);
    
    if (!sendFrame(FUNC_BUS_SERVO, data)) {
        return false;
    }
    
    std::vector<uint8_t> response;
    if (!recvFrame(response, 100)) {
        return false;
    }
    
    // 解析响应: [sub_cmd, count, id, pos_lo, pos_hi]
    if (response.size() >= 5 && response[0] == CMD_READ_POSITION) {
        position = response[3] | (response[4] << 8);
        return true;
    }
    
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
    std::vector<uint8_t> data;
    data.push_back(CMD_READ_ID);
    
    if (!sendFrame(FUNC_BUS_SERVO, data)) {
        return false;
    }
    
    std::vector<uint8_t> response;
    if (!recvFrame(response, 100)) {
        return false;
    }
    
    if (response.size() >= 2 && response[0] == CMD_READ_ID) {
        id = response[1];
        return true;
    }
    
    return false;
}

bool BusServoProtocol::enableTorque(uint8_t servo_id, bool enable)
{
    std::vector<uint8_t> data;
    // 注意: 上电(使能) = CMD_DISABLE_TORQUE (0x0C), 掉电(失能) = CMD_ENABLE_TORQUE (0x0B)
    // 这个命名有点反直觉,但和原始协议一致
    data.push_back(enable ? CMD_DISABLE_TORQUE : CMD_ENABLE_TORQUE);
    data.push_back(1);  // 数量
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
    data.push_back(static_cast<uint8_t>(offset));  // 有符号转无符号
    
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
