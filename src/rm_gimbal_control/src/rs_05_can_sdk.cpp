/**
 * @file rs_05_can_sdk.cpp
 * @brief RS-05 电机 CAN 通信底层 SDK 实现文件
 * @details 提供了各种通信类型指令的组包与解析功能。
 */

#include "rm_gimbal_control/rs_05_can_sdk.hpp"

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
    // 高字节在前，低字节在后 (大端)
    feedback.angle       = (static_cast<int16_t>(frame.data[0]) << 8) | frame.data[1];
    feedback.velocity    = (static_cast<int16_t>(frame.data[2]) << 8) | frame.data[3];
    feedback.torque      = (static_cast<int16_t>(frame.data[4]) << 8) | frame.data[5];
    feedback.temperature = (static_cast<int16_t>(frame.data[6]) << 8) | frame.data[7];
    
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

} // namespace rs_05_can_sdk