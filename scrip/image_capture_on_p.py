#!/usr/bin/env python3
"""
@brief 终端按键抓图工具
@details 订阅 ROS2 图像话题，在远程终端按下 P 键时保存当前最新图像。
"""

import argparse
import os
import select
import sys
import termios
from datetime import datetime

import cv2

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError as e:
    print(f"[错误] 缺少 ROS2 依赖: {e}")
    print("请先 source ROS2 环境后再运行。")
    sys.exit(1)


class TerminalKeyReader:
    """
    @brief 终端单字符读取器
    @details 将终端切换到 cbreak 模式，实现按键即时响应（无需回车）。
    """

    def __init__(self):
        self.fd_ = sys.stdin.fileno()
        self.old_attrs_ = None

    def __enter__(self):
        if not sys.stdin.isatty():
            raise RuntimeError("当前标准输入不是终端，无法监听按键。")
        self.old_attrs_ = termios.tcgetattr(self.fd_)
        new_attrs = termios.tcgetattr(self.fd_)
        new_attrs[3] = new_attrs[3] & ~(termios.ICANON | termios.ECHO)
        termios.tcsetattr(self.fd_, termios.TCSADRAIN, new_attrs)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.old_attrs_ is not None:
            termios.tcsetattr(self.fd_, termios.TCSADRAIN, self.old_attrs_)

    def read_key(self, timeout_sec=0.01):
        """
        @brief 非阻塞读取一个按键
        @param timeout_sec 轮询超时时间（秒）
        @return 读取到的单字符，若无输入则返回空字符串
        """
        readable, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if readable:
            return sys.stdin.read(1)
        return ""


class ImageCaptureOnP(Node):
    """
    @brief 按键抓图节点
    @details 订阅图像并缓存最新一帧，按 P 键时将缓存帧写入磁盘。
    """

    def __init__(self, topic_name, save_dir):
        super().__init__("image_capture_on_p")
        self.bridge_ = CvBridge()
        self.latest_frame_ = None
        self.topic_name_ = topic_name
        self.save_dir_ = save_dir
        os.makedirs(self.save_dir_, exist_ok=True)

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription_ = self.create_subscription(
            Image,
            self.topic_name_,
            self.image_callback,
            sensor_qos,
        )
        self.get_logger().info(f"已订阅图像话题: {self.topic_name_}")
        self.get_logger().info(f"保存目录: {self.save_dir_}")

    def image_callback(self, msg):
        """
        @brief 图像回调
        @param msg sensor_msgs/Image 消息
        """
        try:
            encoding = msg.encoding.lower()
            if encoding in ("mono8", "8uc1"):
                frame = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="mono8")
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame_ = frame
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")

    def save_latest_frame(self):
        """
        @brief 保存最新图像
        @return 成功返回 True，失败返回 False
        """
        if self.latest_frame_ is None:
            self.get_logger().warn("尚未接收到图像，无法保存。")
            return False

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        file_path = os.path.join(self.save_dir_, f"image_{timestamp}.png")
        ok = cv2.imwrite(file_path, self.latest_frame_)
        if ok:
            self.get_logger().info(f"图像已保存: {file_path}")
            return True

        self.get_logger().error(f"图像保存失败: {file_path}")
        return False


def main():
    """
    @brief 程序入口
    """
    parser = argparse.ArgumentParser(description="订阅图像并按 P 键保存当前帧")
    parser.add_argument(
        "--topic",
        type=str,
        default="/image_raw",
        help="订阅的图像话题，默认: /image_raw",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="./captured_images",
        help="图像保存目录，默认: ./captured_images",
    )
    args = parser.parse_args()

    rclpy.init()
    node = ImageCaptureOnP(topic_name=args.topic, save_dir=args.save_dir)

    print("\n按键说明: [P] 保存当前图像, [Q] 退出\n")
    try:
        with TerminalKeyReader() as key_reader:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                key = key_reader.read_key(timeout_sec=0.01)
                if not key:
                    continue
                key_lower = key.lower()
                if key_lower == "p":
                    node.save_latest_frame()
                elif key_lower == "q":
                    node.get_logger().info("收到退出指令，程序结束。")
                    break
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，程序结束。")
    except RuntimeError as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
