/**
 * @file rs_05_can_sdk.cpp
 * @brief RS-05 电机 CAN 通信底层 SDK 实现文件
 * @details 提供了各种通信类型指令的组包与解析功能。
 */

#include "rm_gimbal_control/rs_05_can_sdk.hpp"
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cmath>
#include <iostream>

namespace rs_05_can_sdk {

/**
 * @brief 生成 29 位扩展帧 CAN ID
 * @param comm_type 通信类型
 * @param host_id 主机 ID
 * @param target_id 目标电机 ID
 * @return uint32_t 组装后的 29位 CAN ID
 */
uint32_t Rs05CanSdk::generateCanId(CommType comm_type, uint16_t host_id, uint8_t target_id) {
    uint32_t id = 0;
    id |= (static_cast<uint32_t>(comm_type) & 0x1F) << 24; // Bit 28~24
    id |= (static_cast<uint32_t>(host_id) & 0xFFFF) << 8;  // Bit 23~8
    id |= (static_cast<uint32_t>(target_id) & 0xFF);       // Bit 7~0
    return id;
}

/**
 * @brief 解析 29 位扩展帧 CAN ID
 * @param id 29位 CAN ID
 * @param comm_type 解析出的通信类型
 * @param host_id 解析出的主机 ID
 * @param target_id 解析出的目标电机 CAN ID
 */
void Rs05CanSdk::parseCanId(uint32_t id, uint8_t& comm_type, uint16_t& host_id, uint8_t& target_id) {
    comm_type = (id >> 24) & 0x1F;
    host_id   = (id >> 8) & 0xFFFF;
    target_id = id & 0xFF;
}

/**
 * @brief 生成电机使能帧
 */
CanFrame Rs05CanSdk::buildEnableFrame(uint16_t host_id, uint8_t target_id) {
    CanFrame frame;
    frame.id = generateCanId(CommType::MOTOR_ENABLE, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8); // 数据区全填 0x00
    return frame;
}

/**
 * @brief 生成电机正常停止帧
 */
CanFrame Rs05CanSdk::buildStopFrame(uint16_t host_id, uint8_t target_id) {
    CanFrame frame;
    frame.id = generateCanId(CommType::MOTOR_STOP_CLEAR, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8); // 正常停止全填 0x00
    return frame;
}

/**
 * @brief 生成清除电机错误状态帧
 */
CanFrame Rs05CanSdk::buildClearErrorFrame(uint16_t host_id, uint8_t target_id) {
    CanFrame frame;
    frame.id = generateCanId(CommType::MOTOR_STOP_CLEAR, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    frame.data[0] = 0x01; // Byte[0] = 0x01 代表清除错误
    return frame;
}

/**
 * @brief 生成设置机械零位帧
 */
CanFrame Rs05CanSdk::buildSetZeroFrame(uint16_t host_id, uint8_t target_id) {
    CanFrame frame;
    frame.id = generateCanId(CommType::SET_MECH_ZERO, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    frame.data[0] = 0x01; // Byte[0] = 0x01 设置当前位置为机械零点
    return frame;
}

/**
 * @brief 生成运控模式控制帧 (大端模式)
 */
CanFrame Rs05CanSdk::buildMotionControlFrame(uint16_t host_id, uint8_t target_id, const uint8_t data_8bytes[8]) {
    CanFrame frame;
    frame.id = generateCanId(CommType::MOTION_CTRL, host_id, target_id);
    frame.dlc = 8;
    std::memcpy(frame.data, data_8bytes, 8);
    return frame;
}

/**
 * @brief 解析电机实时反馈帧
 */
bool Rs05CanSdk::parseFeedbackFrame(const CanFrame& frame, MotorFeedback& feedback) {
    uint8_t comm_type;
    uint16_t host_id_part;
    uint8_t target_id;
    
    parseCanId(frame.id, comm_type, host_id_part, target_id);
    
    if (comm_type != static_cast<uint8_t>(CommType::FEEDBACK)) {
        return false;
    }
    
    // bit21~16 包含故障标识 (即 host_id_part 的高 6 位，由于 host_id_part 是 bit23~8，
    // bit21~16 对应 host_id_part 的 bit13~8)
    feedback.fault_flag = (host_id_part >> 8) & 0x3F;
    
    // 数据区域: 当前角度(0~1)、角速度(2~3)、当前力矩(4~5)以及当前温度(6~7)
    // 根据电机的大端字节序，转成有符号16位整数
    feedback.angle       = static_cast<int16_t>((frame.data[0] << 8) | frame.data[1]);
    feedback.velocity    = static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]);
    feedback.torque      = static_cast<int16_t>((frame.data[4] << 8) | frame.data[5]);
    feedback.temperature = static_cast<int16_t>((frame.data[6] << 8) | frame.data[7]);
    
    return true;
}

/**
 * @brief 生成开启/关闭主动上报帧
 */
CanFrame Rs05CanSdk::buildActiveReportFrame(uint16_t host_id, uint8_t target_id, bool enable) {
    CanFrame frame;
    frame.id = generateCanId(CommType::ACTIVE_REPORT, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    // 这里将 F_CMD 设定为 data[0] 作为示例，具体根据协议可修改
    frame.data[0] = enable ? 0x01 : 0x00;
    return frame;
}

/**
 * @brief 生成单个参数读取帧
 */
CanFrame Rs05CanSdk::buildReadParamFrame(uint16_t host_id, uint8_t target_id, uint16_t index) {
    CanFrame frame;
    frame.id = generateCanId(CommType::READ_PARAM, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    
    // Index (低字节在前，高字节在后)
    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    
    return frame;
}

/**
 * @brief 生成单个参数写入帧 (浮点数型数据)
 */
CanFrame Rs05CanSdk::buildWriteParamFrame(uint16_t host_id, uint8_t target_id, uint16_t index, float value) {
    CanFrame frame;
    frame.id = generateCanId(CommType::WRITE_PARAM, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    
    // Index (低字节在前，高字节在后)
    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    
    // Value (Byte4~7 低字节在前，高字节在后)
    uint32_t int_val;
    std::memcpy(&int_val, &value, sizeof(float));
    
    frame.data[4] = int_val & 0xFF;
    frame.data[5] = (int_val >> 8) & 0xFF;
    frame.data[6] = (int_val >> 16) & 0xFF;
    frame.data[7] = (int_val >> 24) & 0xFF;
    
    return frame;
}

/**
 * @brief 生成单个参数写入帧 (整型数据重载)
 */
CanFrame Rs05CanSdk::buildWriteParamFrameInt(uint16_t host_id, uint8_t target_id, uint16_t index, uint32_t value) {
    CanFrame frame;
    frame.id = generateCanId(CommType::WRITE_PARAM, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    
    // Index (低字节在前，高字节在后)
    frame.data[0] = index & 0xFF;
    frame.data[1] = (index >> 8) & 0xFF;
    
    // Value (Byte4~7 低字节在前，高字节在后)
    frame.data[4] = value & 0xFF;
    frame.data[5] = (value >> 8) & 0xFF;
    frame.data[6] = (value >> 16) & 0xFF;
    frame.data[7] = (value >> 24) & 0xFF;
    
    return frame;
}

/**
 * @brief 生成电机数据保存帧
 */
CanFrame Rs05CanSdk::buildSaveParamFrame(uint16_t host_id, uint8_t target_id) {
    CanFrame frame;
    frame.id = generateCanId(CommType::SAVE_PARAM, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    return frame;
}

/**
 * @brief 生成获取设备 ID 帧
 */
CanFrame Rs05CanSdk::buildGetIdFrame(uint16_t host_id, uint8_t target_id) {
    CanFrame frame;
    frame.id = generateCanId(CommType::GET_DEV_ID, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    return frame;
}

/**
 * @brief 生成设置 CAN ID 帧
 */
CanFrame Rs05CanSdk::buildSetCanIdFrame(uint16_t host_id, uint8_t target_id, uint8_t new_id) {
    CanFrame frame;
    frame.id = generateCanId(CommType::SET_CAN_ID, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    // 假设数据段的第一字节写入新的 ID
    frame.data[0] = new_id;
    return frame;
}

/**
 * @brief 生成详细故障反馈帧请求
 */
CanFrame Rs05CanSdk::buildDetailedFaultFrame(uint16_t host_id, uint8_t target_id) {
    CanFrame frame;
    frame.id = generateCanId(CommType::DETAILED_FAULT, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    return frame;
}

/**
 * @brief 生成修改协议类型帧
 */
CanFrame Rs05CanSdk::buildModProtocolFrame(uint16_t host_id, uint8_t target_id, uint8_t protocol_type) {
    CanFrame frame;
    frame.id = generateCanId(CommType::MOD_PROTOCOL, host_id, target_id);
    frame.dlc = 8;
    std::memset(frame.data, 0x00, 8);
    // 假设修改协议类型也是写在数据首字节
    frame.data[0] = protocol_type;
    return frame;
}

RsMotorController::RsMotorController(const std::string& iface, uint16_t master_id, uint8_t motor_id) 
    : iface_(iface), master_id_(master_id), motor_id_(motor_id) {
    init_socket();
}

RsMotorController::~RsMotorController() {
    receive_thread_running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

void RsMotorController::init_socket() {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        perror("socket");
        return;
    }
    struct ifreq ifr{};
    std::strncpy(ifr.ifr_name, iface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        return;
    }
    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return;
    }
    // 两条过滤规则：
    // 规则1：CommType 0x02 实时反馈帧 —— 电机ID 在 Bit15~8
    // 规则2：CommType 0x11/0x12 读写参数响应帧 —— 电机ID 在 Bit7~0（主机ID 在 Bit15~8）
    struct can_filter rfilter[2];
    rfilter[0].can_id   = (static_cast<uint32_t>(motor_id_) << 8) | CAN_EFF_FLAG;
    rfilter[0].can_mask = (0xFFU << 8) | CAN_EFF_FLAG;
    rfilter[1].can_id   = static_cast<uint32_t>(motor_id_) | CAN_EFF_FLAG;
    rfilter[1].can_mask = 0xFFU | CAN_EFF_FLAG;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        perror("setsockopt filter");
    }
}

void RsMotorController::disable_motor(uint8_t clear_error) {
    auto frame = Rs05CanSdk::buildStopFrame(master_id_, motor_id_);
    if(clear_error){
        frame = Rs05CanSdk::buildClearErrorFrame(master_id_, motor_id_);
    }
    send_frame(frame);
}

void RsMotorController::enable_motor() {
    auto frame = Rs05CanSdk::buildEnableFrame(master_id_, motor_id_);
    send_frame(frame);
    
    // 开启主动上报(通信类型 24 / 0x18)，开启类型 2 实时反馈帧的周期性发送
    auto report_frame = Rs05CanSdk::buildActiveReportFrame(master_id_, motor_id_, true);
    send_frame(report_frame);

    std::this_thread::sleep_for(std::chrono::milliseconds(20)); // wait for motor start
    
    // start receiving thread to get motor feedback
    if (!receive_thread_.joinable()) {
        receive_thread_running_ = true;
        receive_thread_ = std::thread(&RsMotorController::receive_loop, this);
    }
}

void RsMotorController::set_mode(uint32_t mode) {
    auto frame = Rs05CanSdk::buildWriteParamFrameInt(master_id_, motor_id_, 0x7005, mode);
    send_frame(frame);
}

uint32_t RsMotorController::get_mode() {
    auto frame = Rs05CanSdk::buildReadParamFrame(master_id_, motor_id_, 0x7005);
    send_frame(frame);
    return 0;
}

double RsMotorController::get_mech_position() {
    auto frame = Rs05CanSdk::buildReadParamFrame(master_id_, motor_id_, 0x7019);
    send_frame(frame);
    return position_.load();
}

void RsMotorController::set_pos_csp(double speed, double angle) {
    auto frame_speed = Rs05CanSdk::buildWriteParamFrame(master_id_, motor_id_, 0x7017, speed);
    send_frame(frame_speed);
    auto frame_angle = Rs05CanSdk::buildWriteParamFrame(master_id_, motor_id_, 0x7016, angle);
    send_frame(frame_angle);
}

void RsMotorController::send_frame(const CanFrame& f) {
    struct can_frame linux_frame{};
    linux_frame.can_id = f.id | CAN_EFF_FLAG;
    linux_frame.can_dlc = f.dlc;
    std::memcpy(linux_frame.data, f.data, 8);
    if (write(socket_fd_, &linux_frame, sizeof(linux_frame)) < 0) {
        // Handle write error silently or log it
    }
}

void RsMotorController::receive_loop() {
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 10000; // 10ms timeout
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    
    while (receive_thread_running_) {
        struct can_frame frame;
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes > 0 && (frame.can_id & CAN_EFF_FLAG)) {
            CanFrame sdk_frame;
            sdk_frame.id = frame.can_id & CAN_EFF_MASK;
            sdk_frame.dlc = frame.can_dlc;
            std::memcpy(sdk_frame.data, frame.data, 8);
            
            uint8_t comm_type_rx;
            uint16_t host_id_rx;
            uint8_t target_id_rx;
            Rs05CanSdk::parseCanId(sdk_frame.id, comm_type_rx, host_id_rx, target_id_rx);

            if (comm_type_rx == static_cast<uint8_t>(CommType::FEEDBACK)) {
                // CommType 0x02 实时反馈帧
                // 对于反馈帧：Bit15~8 = 电机ID，Bit7~0 = 主机ID
                // host_id_rx 实际包含 run_mode(bit23~22) + fault(bit21~16) + motor_id(bit15~8)
                uint8_t motor_id_in_frame = host_id_rx & 0xFF;
                if (motor_id_in_frame != motor_id_) {
                    continue; // 不是本电机的反馈
                }
                MotorFeedback feedback;
                if (Rs05CanSdk::parseFeedbackFrame(sdk_frame, feedback)) {
                    error_code_.store(feedback.fault_flag);
                    // 不再在此处进行软件计圈，多圈角度统一通过 0x7019 参数读取获取
                }
            } else if (comm_type_rx == static_cast<uint8_t>(CommType::READ_PARAM) ||
                       comm_type_rx == static_cast<uint8_t>(CommType::WRITE_PARAM)) {
                // 参数读写应答帧：
                // 不同固件在 ID 字段上可能存在差异，这里同时兼容
                // 1) Bit7~0 为电机ID（target_id_rx）
                // 2) Bit15~8 的低字节为电机ID（host_id_rx & 0xFF）
                const bool is_this_motor =
                    (target_id_rx == motor_id_) || (static_cast<uint8_t>(host_id_rx & 0xFF) == motor_id_);
                if (!is_this_motor) {
                    continue; // 不是本电机的参数应答
                }
                uint16_t index = static_cast<uint16_t>(sdk_frame.data[0]) |
                                 (static_cast<uint16_t>(sdk_frame.data[1]) << 8);
                if (index == 0x7019) {
                    // mechPos 多圈机械角度，小端 float，Byte4~7
                    uint32_t int_val = static_cast<uint32_t>(sdk_frame.data[4])
                                     | (static_cast<uint32_t>(sdk_frame.data[5]) << 8)
                                     | (static_cast<uint32_t>(sdk_frame.data[6]) << 16)
                                     | (static_cast<uint32_t>(sdk_frame.data[7]) << 24);
                    float float_val;
                    std::memcpy(&float_val, &int_val, sizeof(float));
                    double absolute_pos = static_cast<double>(float_val);
                    position_.store(absolute_pos);
                }
            }
        }
    }
}

} // namespace rs_05_can_sdk