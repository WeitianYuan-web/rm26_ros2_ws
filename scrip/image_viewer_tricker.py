#!/usr/bin/env python3
"""
自动瞄准可视化工具 — 订阅图像、检测结果、追踪结果并叠加显示
支持本地窗口显示 + 局域网 MJPEG HTTP 流媒体

订阅话题:
    /image_raw            (sensor_msgs/Image)           — 原始图像
    /detector/armors      (auto_aim_interfaces/Armors)  — 检测到的装甲板
    /tracker/target       (auto_aim_interfaces/Target)  — 追踪目标

用法:
    # 本地窗口显示
    python3 image_viewer_tricker.py

    # 开启网络流 (局域网浏览器访问 http://<IP>:8080)
    python3 image_viewer_tricker.py --stream

    # 仅网络流 (无本地窗口，适合 SSH 无显示器)
    python3 image_viewer_tricker.py --stream --headless

    # 自定义端口、画质、缩放
    python3 image_viewer_tricker.py --stream --port 9090 --stream-quality 60 --stream-scale 0.5

快捷键 (本地窗口):
    q / ESC  — 退出
    s        — 保存当前帧截图
    f        — 显示/隐藏帧率信息
    c        — 显示/隐藏十字准星
    d        — 显示/隐藏检测信息 (装甲板框 + 标签)
    t        — 显示/隐藏追踪信息
    SPACE    — 暂停/恢复显示

依赖:
    pip3 install opencv-python numpy
    (ROS2: rclpy, sensor_msgs, cv_bridge, auto_aim_interfaces)
    (网络流: 无额外依赖，使用 Python 标准库)
"""

import argparse
import math
import os
import socket
import socketserver
import sys
import threading
import time
from datetime import datetime
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2
import numpy as np

# 尝试导入 turbojpeg (比 cv2.imencode 快 2-5 倍)
try:
    from turbojpeg import TurboJPEG
    _tj = TurboJPEG()
    HAS_TURBOJPEG = True
except ImportError:
    HAS_TURBOJPEG = False

# ──────────────────────────────────────────────
#  检测并配置显示器
# ──────────────────────────────────────────────
def ensure_display():
    """
    @brief 检测并确保 DISPLAY 环境变量已设置
    @return True 如果显示器可用，False 如果不可用
    @details 在 Jetson 等嵌入式设备上，通过 SSH 登录时 DISPLAY
             可能未设置，但 X 服务器实际正在运行。此函数会自动
             检测 /tmp/.X11-unix/ 中的 X socket 并设置 DISPLAY。
    """
    # 已经设置了 DISPLAY 或 WAYLAND_DISPLAY
    if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
        return True

    # 检查是否有 X server 的 socket 文件
    x11_dir = "/tmp/.X11-unix"
    if os.path.isdir(x11_dir):
        sockets = sorted(os.listdir(x11_dir))
        for sock in sockets:
            if sock.startswith("X") and sock[1:].isdigit():
                display = f":{sock[1:]}"
                os.environ["DISPLAY"] = display
                print(f"[信息] 自动设置 DISPLAY={display}")
                return True

    return False


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
    print(f"[错误] 缺少 ROS2 基础依赖: {e}")
    print("请确保已 source ROS2 环境: source /opt/ros/<distro>/setup.bash")
    sys.exit(1)

# 尝试导入 auto_aim_interfaces (可能未编译)
try:
    from auto_aim_interfaces.msg import Armors, Target
    HAS_AUTO_AIM = True
except ImportError:
    HAS_AUTO_AIM = False
    print("[警告] 未找到 auto_aim_interfaces, 检测/追踪可视化将不可用")
    print("       请先编译: colcon build --packages-select auto_aim_interfaces")


# ──────────────────────────────────────────────
#  颜色常量
# ──────────────────────────────────────────────
COLOR_GREEN   = (0, 255, 0)
COLOR_RED     = (0, 0, 255)
COLOR_BLUE    = (255, 150, 0)
COLOR_YELLOW  = (0, 255, 255)
COLOR_CYAN    = (255, 255, 0)
COLOR_MAGENTA = (255, 0, 255)
COLOR_WHITE   = (255, 255, 255)
COLOR_ORANGE  = (0, 165, 255)


# ──────────────────────────────────────────────
#  MJPEG HTTP 流媒体服务器 (低延迟优化版)
# ──────────────────────────────────────────────

# 嵌入式 HTML 查看页面 — 使用 JS 手动拉帧替代浏览器原生 MJPEG
# 浏览器原生 <img src="stream"> 会有数秒缓冲，JS 逐帧替换可降至 ~1 帧延迟
_HTML_PAGE = """<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>Auto Aim Stream</title>
<style>
  body {{ margin:0; background:#111; display:flex; flex-direction:column;
         align-items:center; justify-content:center; height:100vh; }}
  img  {{ max-width:100%; max-height:90vh; image-rendering:auto; }}
  .info {{ color:#aaa; font:14px monospace; margin-top:8px; }}
</style>
</head><body>
<img id="view">
<div class="info" id="info">Connecting...</div>
<script>
const img = document.getElementById('view');
const info = document.getElementById('info');
let frames = 0, lastTime = performance.now();

function fetchFrame() {{
    const t0 = performance.now();
    fetch('/snapshot?' + t0)
      .then(r => r.blob())
      .then(blob => {{
        const url = URL.createObjectURL(blob);
        img.onload = () => {{ URL.revokeObjectURL(url); requestAnimationFrame(fetchFrame); }};
        img.onerror = () => {{ URL.revokeObjectURL(url); setTimeout(fetchFrame, 100); }};
        img.src = url;
        frames++;
        const now = performance.now();
        if (now - lastTime > 1000) {{
          const fps = frames * 1000 / (now - lastTime);
          const latency = (now - t0).toFixed(0);
          info.textContent = 'FPS: ' + fps.toFixed(1) + '  Latency: ' + latency + 'ms';
          frames = 0; lastTime = now;
        }}
      }})
      .catch(() => setTimeout(fetchFrame, 200));
}}
fetchFrame();
</script>
</body></html>"""


class ThreadingHTTPServer(socketserver.ThreadingMixIn, HTTPServer):
    """
    @brief 多线程 HTTP 服务器
    @details 每个客户端连接使用独立线程处理，避免阻塞
    """
    daemon_threads = True
    allow_reuse_address = True


class MJPEGStreamHandler(BaseHTTPRequestHandler):
    """
    @brief 低延迟 MJPEG / 快照 HTTP 请求处理器
    @details 支持三种端点:
             /          — HTML 页面 (JS 逐帧拉取，最低延迟)
             /snapshot  — 单帧 JPEG 快照 (供 JS fetch)
             /mjpeg     — 传统 MJPEG 流 (兼容 VLC / img 标签)
    """

    def setup(self):
        """
        @brief 连接初始化，设置 TCP_NODELAY 消除 Nagle 延迟
        """
        super().setup()
        try:
            self.connection.setsockopt(
                socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass

    def do_GET(self):
        if self.path == "/":
            self._serve_html()
        elif self.path.startswith("/snapshot"):
            self._serve_snapshot()
        elif self.path == "/mjpeg":
            self._serve_mjpeg_stream()
        elif self.path == "/status":
            self._serve_status()
        else:
            self.send_error(404)

    def _serve_html(self):
        """
        @brief 返回 HTML 查看页面
        """
        html = _HTML_PAGE.encode()
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def _serve_snapshot(self):
        """
        @brief 返回最新一帧 JPEG 快照 (供 JS 逐帧拉取)
        """
        streamer = self.server.streamer
        jpeg_bytes, _ = streamer.get_jpeg_frame()

        if jpeg_bytes is None:
            self.send_error(503, "No frame available")
            return

        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpeg_bytes)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(jpeg_bytes)

    def _serve_mjpeg_stream(self):
        """
        @brief 传统 MJPEG multipart 流 (兼容 VLC 等)
        """
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()

        streamer = self.server.streamer
        streamer.client_count_ += 1
        streamer.get_logger().info(
            f"MJPEG 客户端连接: {self.client_address[0]} "
            f"(当前 {streamer.client_count_} 个)"
        )
        last_seq = -1

        try:
            while True:
                jpeg_bytes, seq = streamer.get_jpeg_frame()
                if jpeg_bytes is None or seq == last_seq:
                    time.sleep(0.005)
                    continue
                last_seq = seq
                self.wfile.write(b"--frame\r\n"
                                 b"Content-Type: image/jpeg\r\n"
                                 b"Content-Length: " +
                                 str(len(jpeg_bytes)).encode() +
                                 b"\r\n\r\n")
                self.wfile.write(jpeg_bytes)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            streamer.client_count_ -= 1
            streamer.get_logger().info(
                f"MJPEG 客户端断开: {self.client_address[0]} "
                f"(剩余 {streamer.client_count_} 个)"
            )

    def _serve_status(self):
        """
        @brief 返回 JSON 状态信息
        """
        streamer = self.server.streamer
        encoder = "turbojpeg" if HAS_TURBOJPEG else "cv2"
        status = (
            f'{{"clients": {streamer.client_count_}, '
            f'"fps": {streamer.fps_:.1f}, '
            f'"quality": {streamer.jpeg_quality_}, '
            f'"scale": {streamer.scale_}, '
            f'"encoder": "{encoder}"}}'
        )
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(status.encode())

    def log_message(self, format, *args):
        pass


class MJPEGStreamer:
    """
    @brief 低延迟 MJPEG 流媒体管理器
    @details 使用独立编码线程 + 帧序号机制，确保:
             - 编码不阻塞 ROS2 主循环
             - HTTP handler 只发送最新帧，跳过旧帧
             - 无客户端时零 CPU 开销
    """

    def __init__(self, port, quality, scale, logger):
        """
        @brief 初始化流媒体服务器
        @param port HTTP 监听端口
        @param quality JPEG 压缩质量 (1-100)
        @param scale 缩放比例 (0.0-1.0)
        @param logger ROS2 logger
        """
        self.port_ = port
        self.jpeg_quality_ = quality
        self.scale_ = scale
        self.logger_ = logger
        self.client_count_ = 0
        self.fps_ = 0.0

        # 帧缓冲 (JPEG 编码后)
        self._jpeg_lock = threading.Lock()
        self._jpeg_buf = None
        self._frame_seq = 0

        # 原始帧队列 (主线程 -> 编码线程)
        self._raw_lock = threading.Lock()
        self._raw_frame = None
        self._raw_new = threading.Event()

        # 编码线程
        self._encode_thread = threading.Thread(
            target=self._encode_loop, daemon=True)
        self._running = True

        # HTTP 服务器 (多线程)
        self._server = ThreadingHTTPServer(
            ("0.0.0.0", port), MJPEGStreamHandler)
        self._server.streamer = self
        self._serve_thread = threading.Thread(
            target=self._server.serve_forever, daemon=True)

    def get_logger(self):
        return self.logger_

    def start(self):
        """
        @brief 启动编码线程和 HTTP 服务器
        """
        self._encode_thread.start()
        self._serve_thread.start()
        ip = self._get_local_ip()
        self.logger_.info(
            f"流媒体已启动 — 浏览器打开: http://{ip}:{self.port_}/"
        )
        encoder = "turbojpeg" if HAS_TURBOJPEG else "cv2.imencode"
        self.logger_.info(
            f"  画质: {self.jpeg_quality_}  缩放: {self.scale_}  "
            f"编码器: {encoder}"
        )

    def stop(self):
        self._running = False
        self._raw_new.set()
        self._server.shutdown()

    def update_frame(self, frame, fps):
        """
        @brief 提交新帧到编码队列 (非阻塞，由主线程调用)
        @param frame OpenCV BGR 图像
        @param fps 当前帧率
        """
        self.fps_ = fps
        if self.client_count_ <= 0:
            return
        with self._raw_lock:
            self._raw_frame = frame
        self._raw_new.set()

    def get_jpeg_frame(self):
        """
        @brief 获取最新 JPEG 帧和序号 (HTTP handler 调用)
        @return (jpeg_bytes, seq) 元组
        """
        with self._jpeg_lock:
            return self._jpeg_buf, self._frame_seq

    def _encode_loop(self):
        """
        @brief 独立编码线程：从原始帧队列取帧 → 缩放 → JPEG 编码
        """
        while self._running:
            self._raw_new.wait(timeout=0.1)
            self._raw_new.clear()

            with self._raw_lock:
                frame = self._raw_frame
                self._raw_frame = None

            if frame is None:
                continue

            # 缩放
            if self.scale_ < 1.0:
                h, w = frame.shape[:2]
                new_w = int(w * self.scale_)
                new_h = int(h * self.scale_)
                frame = cv2.resize(frame, (new_w, new_h),
                                   interpolation=cv2.INTER_LINEAR)

            # JPEG 编码 (优先 turbojpeg)
            jpeg_bytes = None
            if HAS_TURBOJPEG:
                try:
                    jpeg_bytes = _tj.encode(frame, quality=self.jpeg_quality_)
                except Exception:
                    pass

            if jpeg_bytes is None:
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality_]
                ret, buf = cv2.imencode(".jpg", frame, encode_params)
                if ret:
                    jpeg_bytes = buf.tobytes()

            if jpeg_bytes is not None:
                with self._jpeg_lock:
                    self._jpeg_buf = jpeg_bytes
                    self._frame_seq += 1

    @staticmethod
    def _get_local_ip():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "0.0.0.0"


class ImageViewer(Node):
    """
    @brief 自动瞄准可视化节点
    @details 订阅图像话题、装甲板检测结果和追踪目标，
             将所有信息叠加在画面上实时显示
    """

    def __init__(self, args):
        super().__init__("image_viewer")

        self.bridge_ = CvBridge()
        self.current_frame_ = None
        self.frame_count_ = 0
        self.fps_ = 0.0
        self.last_fps_time_ = time.time()
        self.first_frame_ = True

        # 显示开关
        self.show_fps_ = True
        self.show_detection_ = True
        self.show_tracker_ = True
        self.show_crosshair_ = True
        self.paused_ = False

        # 参数
        self.save_dir_ = args.save_dir
        self.window_width_ = args.width
        self.window_height_ = args.height
        self.topic_name_ = args.topic

        # 网络流
        self.streamer_ = None
        self.headless_ = args.headless
        if args.stream:
            self.streamer_ = MJPEGStreamer(
                port=args.port,
                quality=args.stream_quality,
                scale=args.stream_scale,
                logger=self.get_logger(),
            )

        # 检测/追踪数据缓存
        self.armors_msg_ = None
        self.target_msg_ = None
        self.armors_stamp_ = 0.0
        self.target_stamp_ = 0.0

        if self.save_dir_:
            os.makedirs(self.save_dir_, exist_ok=True)

        # ── QoS 配置 ──
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── 订阅图像 ──
        self.img_sub_ = self.create_subscription(
            Image, self.topic_name_, self.image_callback, sensor_qos,
        )
        self.get_logger().info(f"已订阅图像话题: {self.topic_name_}")

        # ── 订阅检测结果 ──
        if HAS_AUTO_AIM:
            self.armors_sub_ = self.create_subscription(
                Armors, "/detector/armors", self.armors_callback, sensor_qos,
            )
            self.get_logger().info("已订阅检测话题: /detector/armors")

            if not args.no_tracker:
                self.target_sub_ = self.create_subscription(
                    Target, "/tracker/target", self.target_callback, sensor_qos,
                )
                self.get_logger().info("已订阅追踪话题: /tracker/target")

        self.get_logger().info(
            "快捷键: [q/ESC] 退出 | [s] 截图 | [f] 帧率 | "
            "[c] 准星 | [d] 检测 | [t] 追踪 | [SPACE] 暂停"
        )

    # ──────────────────────────────────────────
    #  回调函数
    # ──────────────────────────────────────────
    def image_callback(self, msg):
        """
        @brief 图像话题回调
        @param msg sensor_msgs/Image
        """
        if self.paused_:
            return
        try:
            encoding = msg.encoding
            if encoding in ("mono8", "8UC1"):
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="mono8")
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            elif encoding in ("rgb8", "rgb16"):
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            elif encoding in ("bgr8", "bgr16", "8UC3"):
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            elif encoding in ("bayer_rggb8", "bayer_bggr8", "bayer_gbrg8", "bayer_grbg8"):
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            else:
                cv_image = self.bridge_.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if len(cv_image.shape) == 2:
                    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

            self.current_frame_ = cv_image
            self.frame_count_ += 1
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")

    def armors_callback(self, msg):
        """
        @brief 装甲板检测结果回调
        @param msg auto_aim_interfaces/Armors
        """
        self.armors_msg_ = msg
        self.armors_stamp_ = time.time()

    def target_callback(self, msg):
        """
        @brief 追踪目标回调
        @param msg auto_aim_interfaces/Target
        """
        self.target_msg_ = msg
        self.target_stamp_ = time.time()

    # ──────────────────────────────────────────
    #  绘制函数
    # ──────────────────────────────────────────
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

    def draw_armors(self, display):
        """
        @brief 在画面上绘制检测到的装甲板
        @param display 要绘制的图像
        @details 使用装甲板四个关键点绘制四边形轮廓，
                 并标注类型、编号和3D坐标
        """
        if self.armors_msg_ is None:
            return

        # 超过 0.5s 未更新则认为数据过期
        if time.time() - self.armors_stamp_ > 0.5:
            self.armors_msg_ = None
            return

        for armor in self.armors_msg_.armors:
            # ── 绘制装甲板四边形 ──
            # kpts 顺序: left_light.top, left_light.bottom,
            #            right_light.bottom, right_light.top
            if len(armor.kpts) == 4:
                pts = np.array([
                    [int(armor.kpts[0].x), int(armor.kpts[0].y)],
                    [int(armor.kpts[1].x), int(armor.kpts[1].y)],
                    [int(armor.kpts[2].x), int(armor.kpts[2].y)],
                    [int(armor.kpts[3].x), int(armor.kpts[3].y)],
                ], dtype=np.int32)

                # 根据类型选择颜色
                if armor.type == "large":
                    box_color = COLOR_ORANGE
                else:
                    box_color = COLOR_CYAN

                # 画四边形轮廓
                cv2.polylines(display, [pts], isClosed=True,
                              color=box_color, thickness=2)

                # 画四个关键点
                for i, pt in enumerate(pts):
                    cv2.circle(display, tuple(pt), 4, COLOR_MAGENTA, -1)

                # 画左右灯条中心线
                left_center = ((pts[0] + pts[1]) / 2).astype(int)
                right_center = ((pts[2] + pts[3]) / 2).astype(int)
                cv2.line(display, tuple(left_center), tuple(right_center),
                         COLOR_YELLOW, 1, cv2.LINE_AA)

                # ── 计算标签位置 (四边形上方) ──
                top_y = min(pts[:, 1]) - 10
                center_x = int(np.mean(pts[:, 0]))

                # ── 标注装甲板类型和编号 ──
                label = armor.type.upper()
                if armor.number:
                    label += f" #{armor.number}"

                cv2.putText(
                    display, label,
                    (center_x - 40, max(top_y, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, box_color, 2,
                )

                # ── 标注 3D 坐标 (m) ──
                pos = armor.pose.position
                dist = math.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
                coord_text = f"({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})m"
                dist_text = f"dist: {dist:.2f}m"

                cv2.putText(
                    display, coord_text,
                    (center_x - 70, max(top_y - 18, 15)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, COLOR_WHITE, 1,
                )
                cv2.putText(
                    display, dist_text,
                    (center_x - 40, min(pts[:, 1].max() + 20, display.shape[0] - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_GREEN, 1,
                )

        # 在左下角显示检测到的装甲板数量
        n = len(self.armors_msg_.armors)
        cv2.putText(
            display, f"Armors: {n}",
            (10, display.shape[0] - 15),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, COLOR_CYAN, 2,
        )

    def draw_tracker(self, display):
        """
        @brief 在画面上绘制追踪信息
        @param display 要绘制的图像
        @details 在画面右上角显示追踪状态面板，包含
                 追踪状态、目标ID、3D位置、速度和角速度
        """
        if self.target_msg_ is None:
            return

        # 超过 0.5s 未更新则认为数据过期
        if time.time() - self.target_stamp_ > 0.5:
            self.target_msg_ = None
            return

        target = self.target_msg_
        h, w = display.shape[:2]

        # ── 追踪状态面板 (右上角) ──
        panel_w, panel_h = 280, 170
        panel_x = w - panel_w - 10
        panel_y = 10

        # 半透明背景 — 仅对 ROI 区域混合，避免全图拷贝
        roi = display[panel_y:panel_y + panel_h, panel_x:panel_x + panel_w]
        dark = np.zeros_like(roi)
        cv2.addWeighted(roi, 0.4, dark, 0.6, 0, roi)

        # 面板边框颜色取决于追踪状态
        border_color = COLOR_GREEN if target.tracking else COLOR_RED
        cv2.rectangle(
            display,
            (panel_x, panel_y),
            (panel_x + panel_w, panel_y + panel_h),
            border_color, 2,
        )

        # ── 面板内容 ──
        tx = panel_x + 10
        ty = panel_y + 22
        line_h = 22

        # 追踪状态
        status = "TRACKING" if target.tracking else "LOST"
        status_color = COLOR_GREEN if target.tracking else COLOR_RED
        cv2.putText(
            display, f"Status: {status}", (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55, status_color, 2,
        )
        ty += line_h

        # 目标 ID
        cv2.putText(
            display, f"ID: {target.id}  Armors: {target.armors_num}",
            (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, COLOR_WHITE, 1,
        )
        ty += line_h

        # 3D 位置
        pos = target.position
        cv2.putText(
            display,
            f"Pos: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f})",
            (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX, 0.43, COLOR_YELLOW, 1,
        )
        ty += line_h

        # 速度
        vel = target.velocity
        speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
        cv2.putText(
            display,
            f"Vel: ({vel.x:.2f}, {vel.y:.2f}, {vel.z:.2f})",
            (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX, 0.43, COLOR_YELLOW, 1,
        )
        ty += line_h

        cv2.putText(
            display, f"Speed: {speed:.2f} m/s",
            (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX, 0.43, COLOR_WHITE, 1,
        )
        ty += line_h

        # Yaw 和角速度
        cv2.putText(
            display,
            f"Yaw: {math.degrees(target.yaw):.1f} deg  "
            f"vYaw: {math.degrees(target.v_yaw):.1f} deg/s",
            (tx, ty),
            cv2.FONT_HERSHEY_SIMPLEX, 0.38, COLOR_CYAN, 1,
        )

    def draw_crosshair(self, display):
        """
        @brief 在画面中心绘制十字准星
        @param display 要绘制的图像
        @details 绘制一个带缺口的十字线 + 中心圆点 + 刻度线，
                 模拟 FPS 游戏风格的准星
        """
        h, w = display.shape[:2]
        cx, cy = w // 2, h // 2

        # 准星参数
        gap = 12          # 中心缺口半径
        arm_len = 30      # 十字臂长度
        thickness = 2     # 线宽
        color = COLOR_GREEN

        # 四条十字臂 (带中心缺口)
        # 上
        cv2.line(display, (cx, cy - gap), (cx, cy - gap - arm_len),
                 color, thickness, cv2.LINE_AA)
        # 下
        cv2.line(display, (cx, cy + gap), (cx, cy + gap + arm_len),
                 color, thickness, cv2.LINE_AA)
        # 左
        cv2.line(display, (cx - gap, cy), (cx - gap - arm_len, cy),
                 color, thickness, cv2.LINE_AA)
        # 右
        cv2.line(display, (cx + gap, cy), (cx + gap + arm_len, cy),
                 color, thickness, cv2.LINE_AA)

        # 中心小圆点
        cv2.circle(display, (cx, cy), 3, color, -1, cv2.LINE_AA)

        # 外圈 (虚线效果，用弧线段模拟)
        radius = gap + arm_len + 5
        for angle in range(0, 360, 30):
            start_angle = angle + 5
            end_angle = angle + 25
            cv2.ellipse(display, (cx, cy), (radius, radius),
                        0, start_angle, end_angle, (0, 200, 0), 1, cv2.LINE_AA)

        # 短刻度线 (上下左右 45° 方向)
        tick_len = 8
        for angle_deg in [45, 135, 225, 315]:
            rad = math.radians(angle_deg)
            inner_r = gap + 2
            outer_r = inner_r + tick_len
            x1 = int(cx + inner_r * math.cos(rad))
            y1 = int(cy + inner_r * math.sin(rad))
            x2 = int(cx + outer_r * math.cos(rad))
            y2 = int(cy + outer_r * math.sin(rad))
            cv2.line(display, (x1, y1), (x2, y2), color, 1, cv2.LINE_AA)

    def draw_overlay(self, frame):
        """
        @brief 在画面上叠加所有可视化信息
        @param frame 当前帧图像
        @return 叠加信息后的图像
        """
        display = frame.copy()

        # 绘制十字准星
        if self.show_crosshair_:
            self.draw_crosshair(display)

        # 绘制检测到的装甲板
        if self.show_detection_ and HAS_AUTO_AIM:
            self.draw_armors(display)

        # 绘制追踪信息
        if self.show_tracker_ and HAS_AUTO_AIM:
            self.draw_tracker(display)

        # 绘制基础 HUD 信息
        if self.show_fps_:
            fps_text = f"FPS: {self.fps_:.1f}"
            cv2.putText(
                display, fps_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, COLOR_GREEN, 2,
            )
            cv2.putText(
                display, f"Topic: {self.topic_name_}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_GREEN, 1,
            )
            h, w = frame.shape[:2]
            cv2.putText(
                display, f"{w}x{h}", (10, 85),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_GREEN, 1,
            )

        # 暂停标识
        if self.paused_:
            cv2.putText(
                display, "PAUSED",
                (display.shape[1] // 2 - 80, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, COLOR_RED, 2,
            )

        # 右下角显示功能状态
        status_items = []
        if self.show_crosshair_:
            status_items.append("[C]rosshair: ON")
        else:
            status_items.append("[C]rosshair: OFF")
        if self.show_detection_:
            status_items.append("[D]etect: ON")
        else:
            status_items.append("[D]etect: OFF")
        if self.show_tracker_:
            status_items.append("[T]rack: ON")
        else:
            status_items.append("[T]rack: OFF")

        h_disp = display.shape[0]
        for i, text in enumerate(status_items):
            cv2.putText(
                display, text,
                (display.shape[1] - 160, h_disp - 15 - i * 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1,
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
        @brief 主循环：处理 ROS2 回调 + OpenCV 窗口显示 + 网络流
        @details 支持三种模式:
                 1. 本地窗口 (默认)
                 2. 本地窗口 + 网络流 (--stream)
                 3. 仅网络流 (--stream --headless)
        """
        # 启动流媒体服务器
        if self.streamer_:
            self.streamer_.start()

        # 判断是否需要本地窗口
        use_gui = not self.headless_
        if use_gui and not ensure_display():
            if self.streamer_:
                self.get_logger().warn(
                    "未检测到显示器，自动切换为 headless 模式 (仅网络流)"
                )
                use_gui = False
            else:
                self.get_logger().error(
                    "未检测到可用的显示器。"
                    "请加 --stream --headless 使用纯网络流模式，"
                    "或手动设置: export DISPLAY=:0"
                )
                return

        window_name = None
        if use_gui:
            window_name = f"Auto Aim Viewer - {self.topic_name_}"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

            wait_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(
                wait_frame, f"Waiting for: {self.topic_name_}",
                (30, 220), cv2.FONT_HERSHEY_SIMPLEX, 0.8, COLOR_WHITE, 2,
            )
            cv2.putText(
                wait_frame, "Press [q] to quit",
                (30, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 1,
            )
            cv2.imshow(window_name, wait_frame)
            self.get_logger().info("窗口已打开，等待图像数据...")
        else:
            self.get_logger().info("Headless 模式，等待图像数据...")

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                self.calculate_fps()

                if self.current_frame_ is not None:
                    display = self.draw_overlay(self.current_frame_)

                    # 推送到网络流
                    if self.streamer_:
                        self.streamer_.update_frame(display, self.fps_)

                    # 本地窗口显示
                    if use_gui:
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
                                f"收到图像: {img_w}x{img_h}, 窗口已调整"
                            )
                        cv2.imshow(window_name, display)

                # 键盘事件 (GUI 模式) 或短暂 sleep (headless)
                if use_gui:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord("q") or key == 27:
                        self.get_logger().info("用户退出")
                        break
                    elif key == ord("s"):
                        if self.current_frame_ is not None:
                            self.save_screenshot(self.current_frame_)
                    elif key == ord("f"):
                        self.show_fps_ = not self.show_fps_
                    elif key == ord("c"):
                        self.show_crosshair_ = not self.show_crosshair_
                        state = "开启" if self.show_crosshair_ else "关闭"
                        self.get_logger().info(f"十字准星: {state}")
                    elif key == ord("d"):
                        self.show_detection_ = not self.show_detection_
                        state = "开启" if self.show_detection_ else "关闭"
                        self.get_logger().info(f"检测可视化: {state}")
                    elif key == ord("t"):
                        self.show_tracker_ = not self.show_tracker_
                        state = "开启" if self.show_tracker_ else "关闭"
                        self.get_logger().info(f"追踪可视化: {state}")
                    elif key == ord(" "):
                        self.paused_ = not self.paused_
                        state = "暂停" if self.paused_ else "恢复"
                        self.get_logger().info(f"显示已{state}")

        except KeyboardInterrupt:
            self.get_logger().info("收到中断信号，退出...")
        finally:
            if self.streamer_:
                self.streamer_.stop()
            if use_gui:
                cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="ROS2 自动瞄准可视化工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--topic", "-t", type=str, default="image_raw",
        help="图像话题名称 (默认: image_raw)",
    )
    parser.add_argument(
        "--width", "-W", type=int, default=None,
        help="窗口宽度 (像素)，默认自适应",
    )
    parser.add_argument(
        "--height", "-H", type=int, default=None,
        help="窗口高度 (像素)，默认自适应",
    )
    parser.add_argument(
        "--save-dir", "-d", type=str, default="./screenshots",
        help="截图保存目录 (默认: ./screenshots)",
    )
    parser.add_argument(
        "--no-tracker", action="store_true",
        help="不订阅追踪话题 (仅显示检测结果)",
    )

    # ── 网络流参数 ──
    parser.add_argument(
        "--stream", action="store_true",
        help="开启 MJPEG HTTP 网络流",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="无头模式 (不打开本地窗口，仅网络流)",
    )
    parser.add_argument(
        "--port", "-p", type=int, default=8080,
        help="网络流 HTTP 端口 (默认: 8080)",
    )
    parser.add_argument(
        "--stream-quality", type=int, default=50,
        help="网络流 JPEG 压缩质量 1-100 (默认: 50, 越低越省带宽)",
    )
    parser.add_argument(
        "--stream-scale", type=float, default=0.75,
        help="网络流缩放比例 0.1-1.0 (默认: 0.75, 越小越省 CPU/带宽)",
    )
    args = parser.parse_args()

    # headless 必须搭配 stream
    if args.headless and not args.stream:
        parser.error("--headless 需要搭配 --stream 使用")

    rclpy.init()
    node = ImageViewer(args)

    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
