#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
STM32全向轮机器人控制测试脚本
通过串口(RS485)发送控制帧
控制帧以10Hz频率循环发送，避免超时
"""

import serial
import struct
import time
import argparse
import threading

# 控制帧参数
CONTROL_FRAME_HEADER = 0xAA
CONTROL_FRAME_SIZE = 17  # 1字节帧头 + 4个float(各4字节: vx, vy, vw, feed_rpm)
SEND_FREQUENCY = 10  # 发送频率 (Hz)


class RobotController:
    """机器人控制器类"""
    
    def __init__(self, port, baudrate=921600):
        """
        初始化串口连接
        
        Args:
            port: 串口设备名 (如 'COM3' 或 '/dev/ttyUSB0')
            baudrate: 波特率，默认921600
        """
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.current_vel = (0.0, 0.0, 0.0)  # 当前速度 (vx, vy, vw)
        self.current_feed_rpm = 0.0         # 当前供弹电机转速
        self.send_thread = None
        self.running = False
        
        # 反馈数据初始化
        self.feedback = {
            'x': 0.0, 'y': 0.0, 'theta': 0.0,
            'vx': 0.0, 'vy': 0.0, 'vw': 0.0,
            'feed_rpm': 0.0
        }
        
        print(f"已连接到 {port}, 波特率: {baudrate}")
    
    def _send_frame_once(self, vel_x, vel_y, vel_angular, feed_rpm):
        """
        发送单次控制帧（内部方法）
        
        Args:
            vel_x: x方向线速度 (m/s)
            vel_y: y方向线速度 (m/s)
            vel_angular: 角速度 (rad/s)
            feed_rpm: 供弹电机转速 (RPM)
        """
        # 打包控制帧: 帧头(1B) + vx(4B) + vy(4B) + vw(4B) + feed_rpm(4B)
        frame = struct.pack('<Bffff', CONTROL_FRAME_HEADER, vel_x, vel_y, vel_angular, feed_rpm)
        self.ser.write(frame)

    def _read_feedback(self):
        """读取反馈帧"""
        try:
            # 反馈帧大小: 1B头 + 7个float + 1B尾 = 30字节
            FEEDBACK_FRAME_SIZE = 30
            
            # 尝试读取数据
            if self.ser.in_waiting >= FEEDBACK_FRAME_SIZE:
                # 寻找帧头 0x55
                while self.ser.in_waiting > 0:
                    byte = self.ser.read(1)
                    if byte == b'\x55':
                        # 找到帧头，读取剩余29字节
                        if self.ser.in_waiting >= 29:
                            data = self.ser.read(29)
                            # 校验帧尾
                            if data[-1] == 0xAA:
                                # 解析数据: 7个float (x, y, theta, vx, vy, vw, feed_rpm)
                                floats = struct.unpack('<fffffff', data[:-1])
                                self.feedback['x'] = floats[0]
                                self.feedback['y'] = floats[1]
                                self.feedback['theta'] = floats[2]
                                self.feedback['vx'] = floats[3]
                                self.feedback['vy'] = floats[4]
                                self.feedback['vw'] = floats[5]
                                self.feedback['feed_rpm'] = floats[6]
                            # else:
                                # print("反馈帧尾错误")
                        break
        except Exception as e:
            # print(f"读取反馈出错: {e}")
            pass
    
    def _send_loop(self):
        """后台发送线程"""
        while self.running:
            vx, vy, vw = self.current_vel
            feed_rpm = self.current_feed_rpm
            
            # 发送控制帧
            self._send_frame_once(vx, vy, vw, feed_rpm)
            
            # 尝试读取反馈
            self._read_feedback()
            
            time.sleep(1.0 / SEND_FREQUENCY)
    
    def start_continuous_send(self):
        """启动连续发送"""
        if not self.running:
            self.running = True
            self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
            self.send_thread.start()
    
    def stop_continuous_send(self):
        """停止连续发送"""
        self.running = False
        if self.send_thread:
            self.send_thread.join(timeout=1)
    
    def set_velocity(self, vel_x, vel_y, vel_angular, feed_rpm=0.0, verbose=True):
        """
        设置目标速度（会以10Hz频率持续发送）
        
        Args:
            vel_x: x方向线速度 (m/s)
            vel_y: y方向线速度 (m/s)
            vel_angular: 角速度 (rad/s)
            feed_rpm: 供弹电机转速 (RPM)，默认0
            verbose: 是否打印信息
        """
        self.current_vel = (vel_x, vel_y, vel_angular)
        self.current_feed_rpm = feed_rpm
        if verbose:
            print(f"设置速度: vx={vel_x:.2f} m/s, vy={vel_y:.2f} m/s, vw={vel_angular:.2f} rad/s, feed={feed_rpm:.1f} RPM")
    
    def print_status(self):
        """打印当前反馈状态"""
        f = self.feedback
        # 使用\r覆盖当前行，并在末尾添加输入提示符
        print(f"\r反馈: Pos({f['x']:.2f}, {f['y']:.2f}, {f['theta']:.2f}) "
              f"Vel({f['vx']:.2f}, {f['vy']:.2f}, {f['vw']:.2f}) "
              f"Feed({f['feed_rpm']:.0f})      > ", end="", flush=True)

    def send_for_duration(self, vel_x, vel_y, vel_angular, duration, feed_rpm=0.0):
        """
        发送控制指令持续指定时间
        
        Args:
            vel_x: x方向线速度 (m/s)
            vel_y: y方向线速度 (m/s)
            vel_angular: 角速度 (rad/s)
            duration: 持续时间（秒）
            feed_rpm: 供弹电机转速 (RPM)
        """
        print(f"发送: vx={vel_x:.2f}, vy={vel_y:.2f}, vw={vel_angular:.2f} m/s, feed={feed_rpm:.1f} RPM (持续{duration:.1f}秒)")
        
        start_time = time.time()
        send_count = 0
        
        while time.time() - start_time < duration:
            # 这里的发送由后台线程处理，这里只负责延时和打印
            # self._send_frame_once(vel_x, vel_y, vel_angular, feed_rpm) # 改由后台发送
            self.print_status()
            time.sleep(0.1)
            send_count += 1
        
        print(f"\n  → 测试结束")
    
    def stop(self):
        """停止机器人"""
        self.current_vel = (0.0, 0.0, 0.0)
        self.current_feed_rpm = 0.0
        # 发送停止指令3次确保接收
        for _ in range(3):
            self._send_frame_once(0, 0, 0, 0)
            time.sleep(0.05)
        print("机器人停止")
    
    def close(self):
        """关闭串口"""
        self.stop_continuous_send()
        self.stop()
        self.ser.close()
        print("串口已关闭")


def test_movements(controller, delay=2.0):
    """
    测试各种运动模式
    
    Args:
        controller: 机器人控制器实例
        delay: 每个动作的持续时间(秒)
    """
    print("\n开始测试各种运动模式...")
    print(f"每个指令将以 {SEND_FREQUENCY} Hz 频率循环发送\n")
    
    # 1. 前进
    print("1. 前进 (vx=0.5 m/s)")
    controller.send_for_duration(0.5, 0, 0, delay)
    controller.stop()
    time.sleep(0.5)
    
    # 2. 后退
    print("\n2. 后退 (vx=-0.5 m/s)")
    controller.send_for_duration(-0.5, 0, 0, delay)
    controller.stop()
    time.sleep(0.5)
    
    # 3. 左移
    print("\n3. 左移 (vy=0.5 m/s)")
    controller.send_for_duration(0, 0.5, 0, delay)
    controller.stop()
    time.sleep(0.5)
    
    # 4. 右移
    print("\n4. 右移 (vy=-0.5 m/s)")
    controller.send_for_duration(0, -0.5, 0, delay)
    controller.stop()
    time.sleep(0.5)
    
    # 5. 顺时针旋转
    print("\n5. 顺时针旋转 (vw=-1.0 rad/s)")
    controller.send_for_duration(0, 0, -1.0, delay)
    controller.stop()
    time.sleep(0.5)
    
    # 6. 逆时针旋转
    print("\n6. 逆时针旋转 (vw=1.0 rad/s)")
    controller.send_for_duration(0, 0, 1.0, delay)
    controller.stop()
    time.sleep(0.5)
    
    # 7. 斜向运动
    print("\n7. 斜向运动 (vx=0.3 m/s, vy=0.3 m/s)")
    controller.send_for_duration(0.3, 0.3, 0, delay)
    controller.stop()
    time.sleep(0.5)
    
    # 8. 复合运动（前进+旋转）
    print("\n8. 复合运动 (vx=0.3 m/s, vw=0.5 rad/s)")
    controller.send_for_duration(0.3, 0, 0.5, delay)
    controller.stop()
    
    # 9. 供弹电机测试
    print("\n9. 供弹电机测试 (feed=60 RPM)")
    controller.send_for_duration(0, 0, 0, delay, feed_rpm=60.0)
    controller.stop()
    
    # 10. 移动并发射
    print("\n10. 移动并发射 (vx=0.2 m/s, feed=120 RPM)")
    controller.send_for_duration(0.2, 0, 0, delay, feed_rpm=120.0)
    controller.stop()

    print("\n测试完成!")


def interactive_mode(controller):
    """
    交互式控制模式
    
    Args:
        controller: 机器人控制器实例
    """
    print("\n进入交互式控制模式")
    print(f"控制指令将以 {SEND_FREQUENCY} Hz 频率持续发送")
    print("输入格式: vx vy vw [feed_rpm] (用空格分隔)")
    print("快捷命令:")
    print("  s     - 停止")
    print("  q     - 退出")
    print("示例:")
    print("  0.5 0 0        - 前进0.5m/s")
    print("  0 0 1.0        - 逆时针旋转1.0rad/s")
    print("  0 0 0 60       - 原地供弹60RPM")
    print("  0.3 0 0 120    - 前进并发射")
    
    # 启动连续发送线程
    controller.start_continuous_send()
    print("\n后台发送线程已启动\n")
    
    # 启动自动打印线程
    stop_printing = False
    def auto_print():
        while not stop_printing:
            controller.print_status()
            time.sleep(0.5)
            
    print_thread = threading.Thread(target=auto_print, daemon=True)
    print_thread.start()
    
    try:
        while True:
            try:
                # 提示符已由 print_status 负责显示，这里input不显示提示符
                cmd = input().strip()
                
                if cmd.lower() == 'q':
                    break
                
                if cmd.lower() == 's':
                    controller.set_velocity(0, 0, 0, 0)
                    print("\n已停止") # 换行避免覆盖
                    continue
                
                parts = cmd.split()
                if len(parts) >= 3:
                    vx = float(parts[0])
                    vy = float(parts[1])
                    vw = float(parts[2])
                    feed_rpm = float(parts[3]) if len(parts) > 3 else 0.0
                    
                    controller.set_velocity(vx, vy, vw, feed_rpm)
                elif len(parts) > 0:
                    print("\n输入格式错误！请输入至少三个数字(vx vy vw [feed_rpm])")
                
                # 不需要手动打印状态了
                    
            except ValueError:
                print("\n输入格式错误！请输入数字")
            except EOFError:
                break
                
    except KeyboardInterrupt:
        print("\n接收到中断信号")
    finally:
        stop_printing = True
        controller.stop_continuous_send()
        print("\n后台发送线程已停止")
    
    print("退出交互模式")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(description='STM32全向轮机器人控制测试')
    parser.add_argument('port', help='串口设备 (如 COM3 或 /dev/ttyUSB0)')
    parser.add_argument('-b', '--baudrate', type=int, default=921600, help='波特率 (默认: 921600)')
    parser.add_argument('-t', '--test', action='store_true', help='运行自动测试')
    parser.add_argument('-i', '--interactive', action='store_true', help='交互式控制模式')
    parser.add_argument('-d', '--delay', type=float, default=2.0, help='测试模式中每个动作的持续时间(秒)')
    
    args = parser.parse_args()
    
    try:
        # 创建控制器
        controller = RobotController(args.port, args.baudrate)
        
        if args.test:
            # 自动测试模式
            test_movements(controller, args.delay)
        elif args.interactive:
            # 交互式模式
            interactive_mode(controller)
        else:
            # 默认发送停止命令
            controller.stop()
            print("\n提示: 使用 -t 参数运行自动测试，或使用 -i 参数进入交互模式")
        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        print("\n程序被中断")
    finally:
        if 'controller' in locals():
            controller.close()


if __name__ == '__main__':
    main()
