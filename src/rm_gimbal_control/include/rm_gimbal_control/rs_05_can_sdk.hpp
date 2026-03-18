/**
 * @file rs_05_can_sdk.hpp
 * @brief RS-05 电机 CAN 通信底层 SDK
 * @details 实现了基于该协议的扩展帧 ID 结构定义与各核心功能码的底层 CAN 数据组装和解析。
 * 注意：部分未标明具体字节段的指令（如：运控数据的高低字节拼装、设置 CAN ID 数据段），按照常规形式或者预留空位填入，开发时可根据更具体的协议文档微调。
 */

#ifndef RS_05_CAN_SDK_HPP
#define RS_05_CAN_SDK_HPP

#include <cstdint>
#include <cstring>

namespace rs_05_can_sdk {

/**
 * @brief CAN 通信类型 (功能码) 枚举
 */
enum class CommType : uint8_t {
    GET_DEV_ID          = 0x00, ///< 获取设备 ID
    MOTION_CTRL         = 0x01, ///< 运控模式控制指令
    FEEDBACK            = 0x02, ///< 电机实时反馈
    MOTOR_ENABLE        = 0x03, ///< 电机使能
    MOTOR_STOP_CLEAR    = 0x04, ///< 电机停止 / 清除错误
    SET_MECH_ZERO       = 0x06, ///< 设置机械零位
    SET_CAN_ID          = 0x07, ///< 设置 CAN ID
    READ_PARAM          = 0x11, ///< 单个参数读取
    WRITE_PARAM         = 0x12, ///< 单个参数写入
    DETAILED_FAULT      = 0x15, ///< 详细故障反馈
    SAVE_PARAM          = 0x16, ///< 电机数据保存
    ACTIVE_REPORT       = 0x18, ///< 开启/关闭主动上报
    MOD_PROTOCOL        = 0x19  ///< 修改协议类型
};

/**
 * @brief CAN 帧数据结构
 */
struct CanFrame {
    uint32_t id;       ///< 29位扩展帧 ID
    uint8_t data[8];   ///< 8字节数据区
    uint8_t dlc;       ///< 数据长度，通常为 8
};

/**
 * @brief 电机反馈数据结构
 */
struct MotorFeedback {
    uint8_t fault_flag; ///< 故障标志位 (0: 无故障, 非 0: 有故障，如堵转、过温等)
    int16_t angle;      ///< 当前角度原始值 (需根据协议换算)
    int16_t velocity;   ///< 当前角速度原始值 (需根据协议换算)
    int16_t torque;     ///< 当前力矩原始值 (需根据协议换算)
    int16_t temperature;///< 当前温度原始值 (需根据协议换算)
};

/**
 * @brief RS-05 电机 CAN SDK 核心类
 */
class Rs05CanSdk {
public:
    /**
     * @brief 生成 29 位扩展帧 CAN ID
     * @param comm_type 通信类型 (5 bit, Bit 28~24)
     * @param host_id 主机 ID 或扩展数据段 (16 bit, Bit 23~8)
     * @param target_id 目标电机 CAN ID (8 bit, Bit 7~0)
     * @return uint32_t 组装后的 29位 CAN ID
     */
    static uint32_t generateCanId(CommType comm_type, uint16_t host_id, uint8_t target_id);

    /**
     * @brief 解析 29 位扩展帧 CAN ID
     * @param id 29位 CAN ID
     * @param comm_type 解析出的通信类型
     * @param host_id 解析出的主机 ID / 数据区2
     * @param target_id 解析出的目标电机 CAN ID
     */
    static void parseCanId(uint32_t id, uint8_t& comm_type, uint16_t& host_id, uint8_t& target_id);

    // ==========================================
    // 1. 基础状态控制
    // ==========================================
    
    /**
     * @brief 生成电机使能帧
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildEnableFrame(uint16_t host_id, uint8_t target_id);

    /**
     * @brief 生成电机正常停止帧
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildStopFrame(uint16_t host_id, uint8_t target_id);

    /**
     * @brief 生成清除电机错误状态帧
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildClearErrorFrame(uint16_t host_id, uint8_t target_id);

    /**
     * @brief 生成设置机械零位帧
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildSetZeroFrame(uint16_t host_id, uint8_t target_id);

    // ==========================================
    // 2. 实时控制与数据反馈
    // ==========================================
    
    /**
     * @brief 生成运控模式控制帧 (大端模式)
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @param data_8bytes 包含 目标角度、角速度、力矩、Kp 和 Kd 的 8 字节压缩后数据数组 (需外层处理高字节在前)
     * @return CanFrame 组装好的 CAN 帧
     * @note 具体每个参数位宽因未提供细节，保留字节数组接口形式以便根据 MIT 或特定位宽做封包处理。
     */
    static CanFrame buildMotionControlFrame(uint16_t host_id, uint8_t target_id, const uint8_t data_8bytes[8]);

    /**
     * @brief 解析电机实时反馈帧
     * @param frame 接收到的 CAN 帧 (需要确保高字节在前)
     * @param feedback 解析后的电机状态
     * @return bool 是否成功解析 (判断是否为反馈协议帧)
     */
    static bool parseFeedbackFrame(const CanFrame& frame, MotorFeedback& feedback);

    /**
     * @brief 生成开启/关闭主动上报帧
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @param enable true开启(按指定周期)，false关闭
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildActiveReportFrame(uint16_t host_id, uint8_t target_id, bool enable);

    // ==========================================
    // 3. 参数的读取与写入
    // ==========================================
    
    /**
     * @brief 生成单个参数读取帧
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @param index 参数索引 (低字节在前，小端)
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildReadParamFrame(uint16_t host_id, uint8_t target_id, uint16_t index);

    /**
     * @brief 生成单个参数写入帧 (浮点数型数据)
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @param index 参数索引 (低字节在前)
     * @param value 写入的浮点数据 (低字节在前，高字节在后)
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildWriteParamFrame(uint16_t host_id, uint8_t target_id, uint16_t index, float value);

    /**
     * @brief 生成单个参数写入帧 (整型数据重载)
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @param index 参数索引
     * @param value 写入的整形数据 (低字节在前，高字节在后)
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildWriteParamFrameInt(uint16_t host_id, uint8_t target_id, uint16_t index, uint32_t value);

    /**
     * @brief 生成电机数据保存帧
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildSaveParamFrame(uint16_t host_id, uint8_t target_id);

    // ==========================================
    // 4. 系统与诊断指令
    // ==========================================
    
    /**
     * @brief 生成获取设备 ID 帧
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildGetIdFrame(uint16_t host_id, uint8_t target_id);

    /**
     * @brief 生成设置 CAN ID 帧
     * @param host_id 主机 ID
     * @param target_id 当前电机 ID
     * @param new_id 新的电机 CAN ID
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildSetCanIdFrame(uint16_t host_id, uint8_t target_id, uint8_t new_id);

    /**
     * @brief 生成详细故障反馈帧请求
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildDetailedFaultFrame(uint16_t host_id, uint8_t target_id);

    /**
     * @brief 生成修改协议类型帧
     * @param host_id 主机 ID
     * @param target_id 目标电机 ID
     * @param protocol_type 0:私有协议, 1:CANopen, 2:MIT协议
     * @return CanFrame 组装好的 CAN 帧
     */
    static CanFrame buildModProtocolFrame(uint16_t host_id, uint8_t target_id, uint8_t protocol_type);
};

} // namespace rs_05_can_sdk

#endif // RS_05_CAN_SDK_HPP