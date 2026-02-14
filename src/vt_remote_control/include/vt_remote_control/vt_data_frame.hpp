/**
 * @file vt_data_frame.hpp
 * @brief VTM 图传遥控器数据帧定义与解析
 *
 * @details
 * 数据帧结构（21 字节）：
 *   帧头(2B) + 位域打包数据(17B) + CRC16(2B)
 *
 * UART 参数：921600 baud, 8N1, 无流控
 * 发送周期：14ms / 帧
 */

#ifndef VT_REMOTE_CONTROL__VT_DATA_FRAME_HPP_
#define VT_REMOTE_CONTROL__VT_DATA_FRAME_HPP_

#include <cstdint>
#include <cstring>
#include <string>

namespace vt_remote_control
{

/** @brief 数据帧总长度（字节） */
constexpr uint8_t FRAME_LENGTH = 21;

/** @brief 帧头第一个字节 */
constexpr uint8_t FRAME_HEADER_1 = 0xA9;

/** @brief 帧头第二个字节 */
constexpr uint8_t FRAME_HEADER_2 = 0x53;

/** @brief 通道中值 */
constexpr uint16_t CHANNEL_MID = 1024;

/** @brief 通道最小值 */
constexpr uint16_t CHANNEL_MIN = 364;

/** @brief 通道最大值 */
constexpr uint16_t CHANNEL_MAX = 1684;

/**
 * @brief 挡位切换开关枚举
 */
enum class SwitchMode : uint8_t
{
  C_MODE = 0,  ///< C 档
  N_MODE = 1,  ///< N 档
  S_MODE = 2   ///< S 档
};

/**
 * @brief 键盘按键位映射
 */
enum KeyboardKey : uint16_t
{
  KEY_W     = (1 << 0),
  KEY_S     = (1 << 1),
  KEY_A     = (1 << 2),
  KEY_D     = (1 << 3),
  KEY_SHIFT = (1 << 4),
  KEY_CTRL  = (1 << 5),
  KEY_Q     = (1 << 6),
  KEY_E     = (1 << 7),
  KEY_R     = (1 << 8),
  KEY_F     = (1 << 9),
  KEY_G     = (1 << 10),
  KEY_Z     = (1 << 11),
  KEY_X     = (1 << 12),
  KEY_C     = (1 << 13),
  KEY_V     = (1 << 14),
  KEY_B     = (1 << 15)
};

/**
 * @brief 解析后的遥控器数据结构
 */
struct RemoteData
{
  /** @name 摇杆通道 (364~1684, 中值 1024) */
  ///@{
  uint16_t ch0;  ///< 右摇杆水平方向
  uint16_t ch1;  ///< 右摇杆竖直方向
  uint16_t ch2;  ///< 左摇杆竖直方向
  uint16_t ch3;  ///< 左摇杆水平方向
  ///@}

  /** @name 开关与按键 */
  ///@{
  SwitchMode mode_switch;   ///< 挡位切换开关 (C/N/S)
  uint8_t    pause;         ///< 暂停按键 (0/1)
  uint8_t    fn_left;       ///< 自定义按键（左）(0/1)
  uint8_t    fn_right;      ///< 自定义按键（右）(0/1)
  ///@}

  /** @brief 拨轮 (364~1684, 中值 1024) */
  uint16_t wheel;

  /** @brief 扳机键 (0/1) */
  uint8_t trigger;

  /** @name 鼠标 */
  ///@{
  int16_t mouse_x;       ///< 鼠标 X 轴速度
  int16_t mouse_y;       ///< 鼠标 Y 轴速度
  int16_t mouse_z;       ///< 鼠标滚轮速度
  uint8_t mouse_left;    ///< 鼠标左键 (0/1)
  uint8_t mouse_right;   ///< 鼠标右键 (0/1)
  uint8_t mouse_middle;  ///< 鼠标中键 (0/1)
  ///@}

  /** @brief 键盘按键 (16 位，每位对应一个按键) */
  uint16_t keyboard;

  /**
   * @brief 检查指定键盘按键是否按下
   * @param key 要检查的按键（KeyboardKey 枚举）
   * @return true 按下，false 未按下
   */
  bool isKeyPressed(KeyboardKey key) const
  {
    return (keyboard & static_cast<uint16_t>(key)) != 0;
  }

  /**
   * @brief 获取挡位名称字符串
   * @return 挡位名称 ("C" / "N" / "S" / "UNKNOWN")
   */
  std::string getModeName() const
  {
    switch (mode_switch) {
      case SwitchMode::C_MODE: return "C";
      case SwitchMode::N_MODE: return "N";
      case SwitchMode::S_MODE: return "S";
      default: return "UNKNOWN";
    }
  }
};

// ============================================================================
//  CRC-16 校验（基于参考实现）
// ============================================================================

/** @brief CRC-16 查找表 */
// NOLINTNEXTLINE(modernize-avoid-c-arrays)
static const uint16_t kCrc16Tab[256] = {
  0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
  0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
  0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
  0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
  0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
  0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
  0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
  0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
  0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
  0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
  0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
  0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
  0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
  0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
  0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
  0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
  0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
  0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
  0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
  0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
  0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
  0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
  0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
  0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
  0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
  0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
  0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
  0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
  0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
  0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
  0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
  0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/**
 * @brief 计算 CRC-16 校验和
 * @param p_msg  数据指针
 * @param len    数据长度
 * @param crc16  CRC 初始值（通常 0xFFFF）
 * @return CRC-16 校验值
 */
static inline uint16_t getCrc16CheckSum(const uint8_t *p_msg, uint16_t len, uint16_t crc16)
{
  if (p_msg == nullptr) {
    return 0xFFFF;
  }
  while (len--) {
    uint8_t data = *p_msg++;
    crc16 = static_cast<uint16_t>(crc16 >> 8) ^
            kCrc16Tab[(crc16 ^ static_cast<uint16_t>(data)) & 0x00FF];
  }
  return crc16;
}

/**
 * @brief 验证数据帧的 CRC-16 校验
 * @param p_msg 完整帧数据指针（含 CRC）
 * @param len   帧总长度（含 CRC 的 2 字节）
 * @return true 校验通过，false 校验失败
 */
static inline bool verifyCrc16CheckSum(const uint8_t *p_msg, uint16_t len)
{
  if (p_msg == nullptr || len <= 2) {
    return false;
  }
  uint16_t expected = getCrc16CheckSum(p_msg, len - 2, 0xFFFF);
  return ((expected & 0xFF) == p_msg[len - 2]) &&
         (((expected >> 8) & 0xFF) == p_msg[len - 1]);
}

// ============================================================================
//  数据帧解析
// ============================================================================

/**
 * @brief 验证帧头是否正确
 * @param buf 原始数据缓冲区
 * @return true 帧头匹配，false 不匹配
 */
static inline bool verifyFrameHeader(const uint8_t *buf)
{
  return (buf[0] == FRAME_HEADER_1) && (buf[1] == FRAME_HEADER_2);
}

/**
 * @brief 从原始字节流解析遥控器数据
 *
 * @details 位域布局（从 byte[2] 开始，即 bit offset 16）：
 *   - [16:26]   ch0       11 bits
 *   - [27:37]   ch1       11 bits
 *   - [38:48]   ch2       11 bits
 *   - [49:59]   ch3       11 bits
 *   - [60:61]   mode_sw    2 bits
 *   - [62]      pause      1 bit
 *   - [63]      fn_left    1 bit
 *   - [64]      fn_right   1 bit
 *   - [65:75]   wheel     11 bits
 *   - [76]      trigger    1 bit
 *   - [77:79]   (保留)     3 bits
 *   - [80:95]   mouse_x   16 bits (signed)
 *   - [96:111]  mouse_y   16 bits (signed)
 *   - [112:127] mouse_z   16 bits (signed)
 *   - [128:129] mouse_l    2 bits
 *   - [130:131] mouse_r    2 bits
 *   - [132:133] mouse_m    2 bits
 *   - [134:135] (保留)     2 bits
 *   - [136:151] keyboard  16 bits
 *   - [152:167] crc16     16 bits
 *
 * @param buf  指向 21 字节原始帧数据的指针
 * @param out  解析结果输出
 * @return true 解析成功（帧头 + CRC 均通过），false 解析失败
 */
static inline bool parseFrame(const uint8_t *buf, RemoteData &out)
{
  /* 1. 验证帧头 */
  if (!verifyFrameHeader(buf)) {
    return false;
  }

  /* 2. 验证 CRC */
  if (!verifyCrc16CheckSum(buf, FRAME_LENGTH)) {
    return false;
  }

  /* 3. 按位提取各字段 */

  /* 通道 0: bits [16:26] */
  out.ch0 = static_cast<uint16_t>(
    (buf[2]) | ((buf[3] & 0x07) << 8));

  /* 通道 1: bits [27:37] */
  out.ch1 = static_cast<uint16_t>(
    (buf[3] >> 3) | ((buf[4] & 0x3F) << 5));

  /* 通道 2: bits [38:48] */
  out.ch2 = static_cast<uint16_t>(
    (buf[4] >> 6) | (buf[5] << 2) | ((buf[6] & 0x01) << 10));

  /* 通道 3: bits [49:59] */
  out.ch3 = static_cast<uint16_t>(
    (buf[6] >> 1) | ((buf[7] & 0x0F) << 7));

  /* 挡位切换开关: bits [60:61] */
  out.mode_switch = static_cast<SwitchMode>((buf[7] >> 4) & 0x03);

  /* 暂停按键: bit [62] */
  out.pause = (buf[7] >> 6) & 0x01;

  /* 自定义按键（左）: bit [63] */
  out.fn_left = (buf[7] >> 7) & 0x01;

  /* 自定义按键（右）: bit [64] */
  out.fn_right = buf[8] & 0x01;

  /* 拨轮: bits [65:75] */
  out.wheel = static_cast<uint16_t>(
    (buf[8] >> 1) | ((buf[9] & 0x0F) << 7));

  /* 扳机键: bit [76] */
  out.trigger = (buf[9] >> 4) & 0x01;

  /* 鼠标 X 轴: bits [80:95] (signed int16) */
  out.mouse_x = static_cast<int16_t>(
    static_cast<uint16_t>(buf[10]) | (static_cast<uint16_t>(buf[11]) << 8));

  /* 鼠标 Y 轴: bits [96:111] */
  out.mouse_y = static_cast<int16_t>(
    static_cast<uint16_t>(buf[12]) | (static_cast<uint16_t>(buf[13]) << 8));

  /* 鼠标 Z 轴: bits [112:127] */
  out.mouse_z = static_cast<int16_t>(
    static_cast<uint16_t>(buf[14]) | (static_cast<uint16_t>(buf[15]) << 8));

  /* 鼠标左键: bits [128:129] */
  out.mouse_left = buf[16] & 0x03;

  /* 鼠标右键: bits [130:131] */
  out.mouse_right = (buf[16] >> 2) & 0x03;

  /* 鼠标中键: bits [132:133] */
  out.mouse_middle = (buf[16] >> 4) & 0x03;

  /* 键盘按键: bits [136:151] */
  out.keyboard = static_cast<uint16_t>(
    static_cast<uint16_t>(buf[17]) | (static_cast<uint16_t>(buf[18]) << 8));

  return true;
}

}  // namespace vt_remote_control

#endif  // VT_REMOTE_CONTROL__VT_DATA_FRAME_HPP_
