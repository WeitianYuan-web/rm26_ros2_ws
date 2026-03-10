#!/usr/bin/env python3
"""
@brief 终端按键录视频工具
@details 订阅 ROS2 图像话题，在远程终端按下 P 键开始/停止录制并保存为 MP4。
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


class VideoCaptureOnP(Node):
    """
    @brief 按键录视频节点
    @details 订阅图像并在录制状态下写入 MP4 文件。
    """

    def __init__(self, topic_name, save_dir, fps):
        super().__init__("video_capture_on_p")
        self.bridge_ = CvBridge()
        self.latest_frame_ = None
        self.topic_name_ = topic_name
        self.save_dir_ = save_dir
        self.fps_ = fps

        self.is_recording_ = False
        self.video_writer_ = None
        self.output_path_ = ""
        self.recorded_frames_ = 0

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
        self.get_logger().info(f"目标帧率: {self.fps_} FPS")

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
            return

        if self.is_recording_ and self.video_writer_ is not None:
            self.video_writer_.write(self.latest_frame_)
            self.recorded_frames_ += 1

    def start_recording(self):
        """
        @brief 开始录制
        @return 成功返回 True，失败返回 False
        """
        if self.latest_frame_ is None:
            self.get_logger().warn("尚未接收到图像，无法开始录制。")
            return False
        if self.is_recording_:
            self.get_logger().warn("当前已在录制中。")
            return False

        height, width = self.latest_frame_.shape[:2]
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path_ = os.path.join(self.save_dir_, f"video_{timestamp}.mp4")

        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer_ = cv2.VideoWriter(
            self.output_path_,
            fourcc,
            float(self.fps_),
            (width, height),
            True,
        )
        if not self.video_writer_.isOpened():
            self.video_writer_ = None
            self.get_logger().error("VideoWriter 初始化失败，请检查编码器和输出路径。")
            return False

        self.is_recording_ = True
        self.recorded_frames_ = 0
        self.get_logger().info(f"开始录制: {self.output_path_}")
        return True

    def stop_recording(self):
        """
        @brief 停止录制
        @return 成功返回 True，失败返回 False
        """
        if not self.is_recording_:
            self.get_logger().warn("当前未在录制。")
            return False

        self.is_recording_ = False
        if self.video_writer_ is not None:
            self.video_writer_.release()
            self.video_writer_ = None

        self.get_logger().info(
            f"录制结束: {self.output_path_}，总帧数: {self.recorded_frames_}"
        )
        return True

    def toggle_recording(self):
        """
        @brief 切换录制状态
        """
        if self.is_recording_:
            self.stop_recording()
        else:
            self.start_recording()

    def close(self):
        """
        @brief 资源收尾
        @details 若仍在录制则先停止，确保 MP4 文件完整落盘。
        """
        if self.is_recording_:
            self.get_logger().info("退出前自动停止录制。")
            self.stop_recording()


def main():
    """
    @brief 程序入口
    """
    parser = argparse.ArgumentParser(description="订阅图像并按 P 键开始/停止录制 MP4")
    parser.add_argument(
        "--topic",
        type=str,
        default="/image_raw",
        help="订阅的图像话题，默认: /image_raw",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default="./captured_videos",
        help="视频保存目录，默认: ./captured_videos",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="保存视频帧率，默认: 30.0",
    )
    args = parser.parse_args()

    if args.fps <= 0:
        print("[错误] --fps 必须大于 0")
        sys.exit(1)

    rclpy.init()
    node = VideoCaptureOnP(topic_name=args.topic, save_dir=args.save_dir, fps=args.fps)

    print("\n按键说明: [P] 开始/停止录制, [Q] 退出\n")
    try:
        with TerminalKeyReader() as key_reader:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                key = key_reader.read_key(timeout_sec=0.01)
                if not key:
                    continue
                key_lower = key.lower()
                if key_lower == "p":
                    node.toggle_recording()
                elif key_lower == "q":
                    node.get_logger().info("收到退出指令，程序结束。")
                    break
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，程序结束。")
    except RuntimeError as e:
        node.get_logger().error(str(e))
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
