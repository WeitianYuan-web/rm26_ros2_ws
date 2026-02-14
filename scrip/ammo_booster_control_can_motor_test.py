"""
CAN2 电机控制测试脚本
=====================
通过 CAN 适配器与 CtrBoard-H7 通信，控制电机速度并实时显示反馈数据。

协议:
  - 发送: ID=0x100, Data[0..1] = int16 速度 (RPM, 大端序)
  - 接收: ID=0x101, Data[0..7] = 电机0速度, 电机0电流, 电机1速度, 电机1电流

依赖:
  pip install python-can

用法:
  python ammo_booster_control_can_motor_test.py                          # 默认 slcan, COM3, 1Mbps
  python ammo_booster_control_can_motor_test.py -i slcan -c COM5         # 指定串口
  python ammo_booster_control_can_motor_test.py -i pcan -c PCAN_USBBUS1  # PCAN 适配器
  python ammo_booster_control_can_motor_test.py -i socketcan -c can0     # Linux SocketCAN
"""

import argparse
import struct
import threading
import time
import sys

import can


# ======================== 协议常量 ========================
PC_SPEED_CMD_ID = 0x100   # PC -> MCU: 速度指令
PC_FEEDBACK_ID  = 0x101   # MCU -> PC: 电机状态回传


class MotorFeedback:
    """存储最新的电机反馈数据"""

    def __init__(self):
        self.motor0_speed = 0    # RPM
        self.motor0_torque = 0   # 电流/转矩
        self.motor1_speed = 0    # RPM
        self.motor1_torque = 0   # 电流/转矩
        self.update_count = 0    # 更新计数
        self.last_update = 0.0   # 最后更新时间
        self.lock = threading.Lock()

    def update(self, data: bytes):
        """解析 8 字节反馈数据"""
        if len(data) < 8:
            return
        with self.lock:
            self.motor0_speed  = struct.unpack('>h', data[0:2])[0]
            self.motor0_torque = struct.unpack('>h', data[2:4])[0]
            self.motor1_speed  = struct.unpack('>h', data[4:6])[0]
            self.motor1_torque = struct.unpack('>h', data[6:8])[0]
            self.update_count += 1
            self.last_update = time.time()

    def get(self):
        """线程安全地获取当前数据"""
        with self.lock:
            return (self.motor0_speed, self.motor0_torque,
                    self.motor1_speed, self.motor1_torque,
                    self.update_count, self.last_update)


def send_speed_command(bus: can.BusABC, speed_rpm: int):
    """
    发送速度指令到 MCU

    :param bus: CAN 总线对象
    :param speed_rpm: 目标速度 (int16, RPM)
    """
    speed_rpm = max(-32768, min(32767, speed_rpm))
    data = struct.pack('>h', speed_rpm)
    msg = can.Message(
        arbitration_id=PC_SPEED_CMD_ID,
        data=data,
        is_extended_id=False,
    )
    try:
        bus.send(msg)
        return True
    except can.CanError as e:
        print(f"\n[错误] CAN 发送失败: {e}")
        return False


def receive_thread(bus: can.BusABC, feedback: MotorFeedback, stop_event: threading.Event):
    """后台接收线程，持续接收 CAN 反馈数据"""
    while not stop_event.is_set():
        msg = bus.recv(timeout=0.1)
        if msg is not None and msg.arbitration_id == PC_FEEDBACK_ID:
            feedback.update(msg.data)


def display_thread(feedback: MotorFeedback, stop_event: threading.Event):
    """后台显示线程，每 200ms 刷新一次状态"""
    while not stop_event.is_set():
        m0_spd, m0_trq, m1_spd, m1_trq, count, last_t = feedback.get()
        dt = time.time() - last_t if last_t > 0 else float('inf')

        # 计算接收频率
        freq_str = f"{1.0/dt:.0f} Hz" if 0 < dt < 1.0 else "---"

        status = (
            f"\r  电机0: {m0_spd:+6d} RPM, 电流: {m0_trq:+6d}"
            f"  |  电机1: {m1_spd:+6d} RPM, 电流: {m1_trq:+6d}"
            f"  |  帧数: {count}  频率: {freq_str}   "
        )
        sys.stdout.write(status)
        sys.stdout.flush()
        time.sleep(0.2)


def interactive_mode(bus: can.BusABC, feedback: MotorFeedback, stop_event: threading.Event):
    """交互模式：键盘输入速度"""
    current_speed = 500  # 初始速度与 MCU 一致

    print("=" * 70)
    print("  CAN2 电机控制测试工具")
    print("=" * 70)
    print(f"  发送 ID: 0x{PC_SPEED_CMD_ID:03X}   接收 ID: 0x{PC_FEEDBACK_ID:03X}")
    print("-" * 70)
    print("  命令:")
    print("    <数字>     - 设置速度 (RPM), 如: 1000, -500, 0")
    print("    s          - 停止电机 (速度=0)")
    print("    +/-        - 速度 +100 / -100 RPM")
    print("    q          - 退出")
    print("-" * 70)
    print(f"  当前初始速度: {current_speed} RPM (MCU 上电默认)")
    print()

    while not stop_event.is_set():
        try:
            print()  # 换行避免和状态行重叠
            user_input = input("  输入速度指令 > ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n  退出中...")
            break

        if not user_input:
            continue

        if user_input.lower() == 'q':
            print("  发送停止指令并退出...")
            send_speed_command(bus, 0)
            break
        elif user_input.lower() == 's':
            current_speed = 0
            if send_speed_command(bus, current_speed):
                print(f"  >> 已发送: 停止电机")
        elif user_input == '+':
            current_speed += 100
            current_speed = min(current_speed, 9000)
            if send_speed_command(bus, current_speed):
                print(f"  >> 已发送: {current_speed} RPM")
        elif user_input == '-':
            current_speed -= 100
            current_speed = max(current_speed, -9000)
            if send_speed_command(bus, current_speed):
                print(f"  >> 已发送: {current_speed} RPM")
        else:
            try:
                current_speed = int(user_input)
                if send_speed_command(bus, current_speed):
                    print(f"  >> 已发送: {current_speed} RPM")
            except ValueError:
                print(f"  [!] 无效输入: '{user_input}'")

    stop_event.set()


def main():
    parser = argparse.ArgumentParser(description="CAN2 电机控制测试脚本")
    parser.add_argument('-i', '--interface', default='slcan',
                        help='CAN 接口类型 (slcan/pcan/socketcan/canable 等, 默认: slcan)')
    parser.add_argument('-c', '--channel', default='COM3',
                        help='CAN 通道 (串口号或设备名, 默认: COM3)')
    parser.add_argument('-b', '--bitrate', type=int, default=1000000,
                        help='CAN 波特率 (默认: 1000000)')
    args = parser.parse_args()

    print(f"\n  正在连接 CAN 适配器: {args.interface} @ {args.channel}, {args.bitrate} bps ...")

    try:
        bus = can.Bus(
            interface=args.interface,
            channel=args.channel,
            bitrate=args.bitrate,
        )
    except Exception as e:
        print(f"\n  [错误] 无法打开 CAN 接口: {e}")
        print(f"  请检查:")
        print(f"    1) CAN 适配器是否连接")
        print(f"    2) 串口号/通道名是否正确")
        print(f"    3) 驱动是否安装")
        print(f"    4) python-can 是否安装: pip install python-can")
        sys.exit(1)

    print(f"  CAN 接口已打开\n")

    feedback = MotorFeedback()
    stop_event = threading.Event()

    # 启动接收线程
    rx_thread = threading.Thread(target=receive_thread, args=(bus, feedback, stop_event), daemon=True)
    rx_thread.start()

    # 启动显示线程
    disp_thread = threading.Thread(target=display_thread, args=(feedback, stop_event), daemon=True)
    disp_thread.start()

    # 进入交互模式
    try:
        interactive_mode(bus, feedback, stop_event)
    except KeyboardInterrupt:
        print("\n  Ctrl+C 退出...")
    finally:
        stop_event.set()
        # 发送停止指令
        try:
            send_speed_command(bus, 0)
        except Exception:
            pass
        bus.shutdown()
        print("  CAN 接口已关闭, 电机已停止\n")


if __name__ == '__main__':
    main()
