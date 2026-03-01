#!/usr/bin/env python3
"""
图像话题可视化工具 — 订阅 ROS2 Image 话题并在窗口中实时显示

用法:
    # 默认订阅 image_raw 话题
    python3 image_viewer_GStreamer.py

    # 指定话题名称
    python3 image_viewer_GStreamer.py --topic /camera/image_raw

    # 指定窗口大小
    python3 image_viewer_GStreamer.py --width 1280 --height 720

    # 保存截图 (按 's' 键)
    python3 image_viewer_GStreamer.py --save-dir ./screenshots

    # 启用 GStreamer 低延迟 UDP/RTP 推流
    python3 image_viewer_GStreamer.py --stream --stream-host 192.168.60.94 --stream-port 5600 --stream-hw

    # 无界面推流模式（适合远端部署）
    python3 image_viewer_GStreamer.py --headless --stream --stream-host 192.168.60.94 --stream-port 5600

接收端示例:
    # H264
    gst-launch-1.0 -v udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" \
      ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false

快捷键:
    q / ESC  — 退出
    s        — 保存当前帧截图
    f        — 显示/隐藏帧率信息
    SPACE    — 暂停/恢复显示

依赖:
    pip3 install opencv-python numpy
    (ROS2: rclpy, sensor_msgs, cv_bridge)
    (可选推流: OpenCV 需启用 GStreamer)
"""

import argparse
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np

# ──────────────────────────────────────────────
#  检测显示器是否可用
# ──────────────────────────────────────────────
def has_display():
    """检测当前环境是否有可用的显示器"""
    display = os.environ.get("DISPLAY", "")
    wayland = os.environ.get("WAYLAND_DISPLAY", "")
    return bool(display or wayland)


class GStreamerVideoStreamer:
    """
    @brief 基于 GStreamer 的 UDP/RTP 视频推流器
    """

    def __init__(self, host, port, bitrate_kbps, fps, codec, use_hw):
        self.host_ = host
        self.port_ = port
        self.bitrate_kbps_ = bitrate_kbps
        self.fps_ = fps
        self.codec_ = codec
        self.use_hw_ = use_hw
        self.writer_ = None
        self.frame_size_ = None

    def _build_pipeline(self, width, height):
        if self.codec_ == "h265":
            if self.use_hw_:
                encoder = (
                    f"nvv4l2h265enc bitrate={self.bitrate_kbps_ * 1000} "
                    "insert-sps-pps=true iframeinterval=30 idrinterval=30 "
                    "control-rate=1 maxperf-enable=1 preset-level=1"
                )
            else:
                encoder = (
                    f"x265enc bitrate={self.bitrate_kbps_} speed-preset=ultrafast "
                    "tune=zerolatency key-int-max=30"
                )
            payloader = "rtph265pay config-interval=1 pt=96"
        else:
            if self.use_hw_:
                encoder = (
                    f"nvv4l2h264enc bitrate={self.bitrate_kbps_ * 1000} "
                    "insert-sps-pps=true iframeinterval=30 idrinterval=30 "
                    "control-rate=1 maxperf-enable=1 preset-level=1"
                )
            else:
                encoder = (
                    f"x264enc bitrate={self.bitrate_kbps_} speed-preset=ultrafast "
                    "tune=zerolatency key-int-max=30"
                )
            payloader = "rtph264pay config-interval=1 pt=96"

        pre_encoder = "videoconvert ! "
        if self.use_hw_:
            pre_encoder = (
                "videoconvert ! video/x-raw,format=I420 ! "
                "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! "
            )

        return (
            "appsrc is-live=true do-timestamp=true format=time "
            f"caps=video/x-raw,format=BGR,width={width},height={height},framerate={self.fps_}/1 ! "
            "queue leaky=downstream max-size-buffers=2 ! "
            f"{pre_encoder}"
            f"{encoder} ! "
            f"{payloader} ! "
            f"udpsink host={self.host_} port={self.port_} sync=false async=false"
        )

    def start(self, width, height):
        self.stop()
        pipeline = self._build_pipeline(width, height)
        writer = cv2.VideoWriter(
            pipeline,
            cv2.CAP_GSTREAMER,
            0,
            float(self.fps_),
            (int(width), int(height)),
            True,
        )
        if not writer.isOpened():
            return False

        self.writer_ = writer
        self.frame_size_ = (int(width), int(height))
        return True

    def push_frame(self, frame):
        if frame is None:
            return
        h, w = frame.shape[:2]
        target_size = (w, h)

        if self.writer_ is None or self.frame_size_ != target_size:
            ok = self.start(w, h)
            if not ok:
                return

        self.writer_.write(frame)

    def stop(self):
        if self.writer_ is not None:
            self.writer_.release()
            self.writer_ = None
            self.frame_size_ = None


# ──────────────────────────────────────────────
#  ROS2 导入
# ──────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile,
        ReliabilityPolicy,
        HistoryPolicy,
        DurabilityPolicy,
    )
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except ImportError as e:
    print(f"[错误] 缺少 ROS2 依赖: {e}")
    print("请确保已 source ROS2 环境: source /opt/ros/<distro>/setup.bash")
    sys.exit(1)


class ImageViewer(Node):
    """
    @brief 图像可视化节点，订阅 Image 话题并在 OpenCV 窗口中显示
    """

    def __init__(self, args):
        super().__init__("image_viewer")

        self.bridge_ = CvBridge()
        self.current_frame_ = None
        self.frame_count_ = 0
        self.fps_ = 0.0
        self.last_fps_time_ = time.time()
        self.show_fps_ = True
        self.paused_ = False
        self.save_dir_ = args.save_dir
        self.window_width_ = args.width
        self.window_height_ = args.height
        self.topic_name_ = args.topic
        self.first_frame_ = True  # 标记是否为第一帧，用于自适应窗口
        self.headless_ = args.headless
        self.stream_enabled_ = args.stream
        self.streamer_ = None

        # 如果指定了保存目录，确保目录存在
        if self.save_dir_:
            os.makedirs(self.save_dir_, exist_ok=True)

        if self.stream_enabled_:
            self.streamer_ = GStreamerVideoStreamer(
                host=args.stream_host,
                port=args.stream_port,
                bitrate_kbps=args.stream_bitrate,
                fps=args.stream_fps,
                codec=args.stream_codec,
                use_hw=args.stream_hw,
            )
            self.get_logger().info(
                f"GStreamer推流已启用: {args.stream_host}:{args.stream_port}, "
                f"codec={args.stream_codec}, bitrate={args.stream_bitrate}kbps, "
                f"fps={args.stream_fps}, hw={'on' if args.stream_hw else 'off'}"
            )

        # 使用 SensorDataQoS 兼容的 QoS 配置，与发布者匹配
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

        self.get_logger().info(f"已订阅话题: {self.topic_name_}")
        self.get_logger().info("快捷键: [q/ESC] 退出 | [s] 截图 | [f] 帧率 | [SPACE] 暂停")

    def image_callback(self, msg):
        """
        @brief 图像话题回调函数
        @param msg sensor_msgs/Image 消息
        """
        if self.paused_:
            return

        try:
            # 自动检测编码格式并转换
            encoding = msg.encoding
            if encoding in ("mono8", "8UC1"):
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="mono8")
                # 转换为 BGR 以便在窗口上叠加彩色文字
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            elif encoding in ("rgb8", "rgb16"):
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            elif encoding in ("bgr8", "bgr16", "8UC3"):
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            elif encoding in ("bayer_rggb8", "bayer_bggr8", "bayer_gbrg8", "bayer_grbg8"):
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                # 通用 passthrough
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if len(cv_image.shape) == 2:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            self.current_frame_ = cv_image
            self.frame_count_ += 1

        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")

    def calculate_fps(self):
        """
        @brief 计算并更新帧率
        """
        now = time.time()
        elapsed = now - self.last_fps_time_
        if elapsed >= 1.0:
            self.fps_ = self.frame_count_ / elapsed
            self.frame_count_ = 0
            self.last_fps_time_ = now

    def draw_overlay(self, frame):
        """
        @brief 在画面上叠加信息 (帧率、话题名称等)
        @param frame 当前帧图像
        @return 叠加信息后的图像
        """
        display = frame.copy()

        if self.show_fps_:
            # 帧率信息
            fps_text = f"FPS: {self.fps_:.1f}"
            cv2.putText(
                display, fps_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2,
            )

            # 话题名称
            cv2.putText(
                display, f"Topic: {self.topic_name_}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )

            # 分辨率
            h, w = frame.shape[:2]
            cv2.putText(
                display, f"{w}x{h}", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
            )

        if self.paused_:
            cv2.putText(
                display, "PAUSED", (display.shape[1] // 2 - 80, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2,
            )

        return display

    def save_screenshot(self, frame):
        """
        @brief 保存当前帧为截图
        @param frame 当前帧图像
        """
        save_dir = self.save_dir_ if self.save_dir_ else "."
        os.makedirs(save_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = os.path.join(save_dir, f"screenshot_{timestamp}.png")
        cv2.imwrite(filename, frame)
        self.get_logger().info(f"截图已保存: {filename}")

    def run(self):
        """
        @brief 主循环：处理 ROS2 回调 + OpenCV 窗口显示
        """
        display_ready = has_display() and not self.headless_
        if not display_ready and not self.stream_enabled_:
            self.get_logger().error(
                "未检测到显示器 (DISPLAY 未设置)。"
                "请在有图形界面的环境中运行此脚本，"
                "或通过 X11 转发 (ssh -X) 连接，"
                "或启用 --stream 进行无界面推流。"
            )
            return

        window_name = f"Image Viewer - {self.topic_name_}"
        if display_ready:
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
            wait_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                wait_frame, f"Waiting for topic: {self.topic_name_}",
                (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2,
            )
            cv2.imshow(window_name, wait_frame)
            self.get_logger().info("窗口已打开，等待图像数据...")
        else:
            self.get_logger().info("已进入无界面模式，仅进行ROS处理与推流")

        try:
            while rclpy.ok():
                # 处理 ROS2 回调
                rclpy.spin_once(self, timeout_sec=0.01)

                # 计算帧率
                self.calculate_fps()

                # 显示图像
                if self.current_frame_ is not None:
                    display = self.draw_overlay(self.current_frame_)

                    if self.stream_enabled_ and self.streamer_ is not None:
                        self.streamer_.push_frame(display)

                    if display_ready:
                        if self.first_frame_:
                            self.first_frame_ = False
                            img_h, img_w = display.shape[:2]
                            if self.window_width_ and self.window_height_:
                                cv2.resizeWindow(
                                    window_name,
                                    self.window_width_,
                                    self.window_height_,
                                )
                            else:
                                max_w, max_h = 1280, 960
                                scale = min(max_w / img_w, max_h / img_h, 1.0)
                                win_w = int(img_w * scale)
                                win_h = int(img_h * scale)
                                cv2.resizeWindow(window_name, win_w, win_h)
                            self.get_logger().info(
                                f"收到图像: {img_w}x{img_h}, "
                                f"窗口已调整"
                            )

                        cv2.imshow(window_name, display)

                if display_ready:
                    key = cv2.waitKey(1) & 0xFF

                    if key == ord("q") or key == 27:
                        self.get_logger().info("用户退出")
                        break
                    elif key == ord("s"):
                        if self.current_frame_ is not None:
                            self.save_screenshot(self.current_frame_)
                    elif key == ord("f"):
                        self.show_fps_ = not self.show_fps_
                    elif key == ord(" "):
                        self.paused_ = not self.paused_
                        state = "暂停" if self.paused_ else "恢复"
                        self.get_logger().info(f"显示已{state}")

        except KeyboardInterrupt:
            self.get_logger().info("收到中断信号，退出...")
        finally:
            if self.streamer_ is not None:
                self.streamer_.stop()
            if display_ready:
                cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="ROS2 图像话题可视化工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--topic", "-t",
        type=str,
        default="/image_raw",
        help="要订阅的图像话题名称 (默认: /image_raw)",
    )
    parser.add_argument(
        "--width", "-W",
        type=int,
        default=None,
        help="窗口宽度 (像素)，默认自适应",
    )
    parser.add_argument(
        "--height", "-H",
        type=int,
        default=None,
        help="窗口高度 (像素)，默认自适应",
    )
    parser.add_argument(
        "--save-dir", "-d",
        type=str,
        default="./screenshots",
        help="截图保存目录 (默认: ./screenshots)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="无界面模式，不创建OpenCV窗口",
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="启用GStreamer UDP/RTP视频推流",
    )
    parser.add_argument(
        "--stream-host",
        type=str,
        default="127.0.0.1",
        help="推流目标IP (默认: 127.0.0.1)",
    )
    parser.add_argument(
        "--stream-port",
        type=int,
        default=5600,
        help="推流目标端口 (默认: 5600)",
    )
    parser.add_argument(
        "--stream-bitrate",
        type=int,
        default=4000,
        help="推流码率kbps (默认: 4000)",
    )
    parser.add_argument(
        "--stream-fps",
        type=int,
        default=30,
        help="推流帧率 (默认: 30)",
    )
    parser.add_argument(
        "--stream-codec",
        type=str,
        choices=["h264", "h265"],
        default="h264",
        help="推流编码格式 (默认: h264)",
    )
    parser.add_argument(
        "--stream-hw",
        action="store_true",
        help="使用硬件编码器 (Jetson推荐)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = ImageViewer(args)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
