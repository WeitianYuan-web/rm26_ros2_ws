#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""!
@file app.py
@brief RM26 机器人监控网页后端
@details
- 使用 Flask 提供局域网可访问网页
- 使用 rclpy 监听 ROS2 话题并缓存状态
- 图像采用“手动刷新”模式，避免持续高频处理占用资源
"""

import threading
import time
from datetime import datetime
from io import BytesIO
from typing import Any, Dict, List, Optional

from flask import Flask, Response, jsonify, render_template

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rosidl_runtime_py.utilities import get_message

try:
    import cv2
    from cv_bridge import CvBridge
except Exception:  # pragma: no cover
    cv2 = None
    CvBridge = None


TOPICS_TO_MONITOR = [
    "/ammo_booster/feedback",
    "/camera_info",
    "/chassis/command",
    "/chassis/feedback",
    "/chassis/gyro_z",
    "/detector/armors",
    "/gimbal/cmd",
    "/gimbal/gyro_z",
    "/image_raw",
    "/motor1/multi_turn_position",
    "/motor1/target_position",
    "/motor2/multi_turn_position",
    "/motor2/target_position",
    "/parameter_events",
    "/rosout",
    "/vt_remote/channels",
    "/vt_remote/key_toggles",
    "/vt_remote/keyboard",
    "/vt_remote/keyboard_readable",
    "/vt_remote/keyboard_toggles",
    "/vt_remote/mouse",
    "/vt_remote/mouse_toggles",
    "/vt_remote/switches",
]

HEAVY_TOPICS = {"/rosout", "/parameter_events", "/image_raw"}
IMAGE_TOPIC = "/image_raw"


class MonitorNode(Node):
    """!
    @brief ROS2 监控节点，负责订阅并缓存话题状态
    @details 轻量版实现：仅跟踪在线状态和简要预览数据，避免高频回调导致高CPU占用。
             移除频率计数和HZ计算，大幅增加采样间隔。
    """

    def __init__(self, topics: List[str]) -> None:
        super().__init__("rm26_web_monitor_node")
        self._topics = set(topics)
        self._lock = threading.Lock()
        self._subscribers: Dict[str, Any] = {}
        self._topic_types: Dict[str, str] = {}
        self._topic_data: Dict[str, Dict[str, Any]] = {}
        self._last_preview_update: Dict[str, float] = {}
        self._last_sample_update: Dict[str, float] = {}

        self._image_request_event = threading.Event()
        self._image_capture_event = threading.Event()
        self._latest_image_msg = None
        self._image_subscriber = None
        self._image_sub_lock = threading.Lock()
        self._image_msg_cls = None
        self._bridge = CvBridge() if CvBridge is not None else None

        for topic in self._topics:
            self._topic_data[topic] = self._init_topic_entry(topic)
            self._last_preview_update[topic] = 0.0
            self._last_sample_update[topic] = 0.0

        self._discovery_timer = self.create_timer(2.0, self._discover_and_subscribe)

    def _init_topic_entry(self, topic: str) -> Dict[str, Any]:
        """!
        @brief 初始化话题数据条目
        @details 轻量化版本：移除 count/hz 字段，减少内存和计算开销
        """
        return {
            "topic": topic,
            "type": "unknown",
            "online": False,
            "last_recv_sec": None,
            "last_recv_str": "-",
            "preview": "等待消息...",
        }

    def _discover_and_subscribe(self) -> None:
        available = dict(self.get_topic_names_and_types())
        for topic in self._topics:
            # 图像话题改为按需订阅，避免高频图像回调长期占用CPU
            if topic == IMAGE_TOPIC:
                if topic in available and available[topic]:
                    type_str = available[topic][0]
                    with self._lock:
                        self._topic_data[topic]["type"] = type_str
                    if self._image_msg_cls is None:
                        try:
                            self._image_msg_cls = get_message(type_str)
                        except Exception:
                            pass
                continue

            if topic in self._subscribers:
                continue
            if topic not in available or not available[topic]:
                continue

            type_str = available[topic][0]
            try:
                msg_cls = get_message(type_str)
            except Exception as exc:
                self.get_logger().warning(f"无法加载消息类型 {type_str}: {exc}")
                continue

            qos = self._choose_qos(topic)
            callback = self._make_callback(topic)
            try:
                sub = self.create_subscription(msg_cls, topic, callback, qos)
            except Exception as exc:
                self.get_logger().warning(f"订阅失败 {topic} [{type_str}]: {exc}")
                continue

            with self._lock:
                self._subscribers[topic] = sub
                self._topic_types[topic] = type_str
                self._topic_data[topic]["type"] = type_str

            self.get_logger().info(f"已订阅 {topic} [{type_str}]")

    def _choose_qos(self, topic: str) -> QoSProfile:
        if topic in ("/image_raw", "/camera_info"):
            return QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
            )
        return QoSProfile(depth=10)

    def _make_callback(self, topic: str):
        """!
        @brief 创建话题回调函数
        @details 轻量优化：大幅增加采样间隔（0.5s+），移除hz/count计算，减少CPU占用。
                 仅在必要时更新预览数据。
        """
        def _callback(msg) -> None:
            now = time.time()
            # 轻量化：大幅增加采样间隔，避免高频话题占用过多CPU
            sample_interval = 0.5 if topic in HEAVY_TOPICS else 0.2
            if (now - self._last_sample_update[topic]) < sample_interval:
                if topic == "/image_raw" and self._image_request_event.is_set():
                    self._latest_image_msg = msg
                    self._image_capture_event.set()
                return

            preview_interval = 2.0 if topic in HEAVY_TOPICS else 1.0
            update_preview = (now - self._last_preview_update[topic]) >= preview_interval

            with self._lock:
                entry = self._topic_data[topic]
                entry["online"] = True
                entry["last_recv_sec"] = now

                if update_preview:
                    entry["preview"] = self._make_preview(topic, msg)
                    self._last_preview_update[topic] = now
                self._last_sample_update[topic] = now

            if topic == "/image_raw" and self._image_request_event.is_set():
                self._latest_image_msg = msg
                self._image_capture_event.set()

        return _callback

    def _image_callback(self, msg) -> None:
        now = time.time()
        with self._lock:
            entry = self._topic_data[IMAGE_TOPIC]
            entry["online"] = True
            entry["last_recv_sec"] = now
            if (now - self._last_preview_update[IMAGE_TOPIC]) >= 2.0:
                entry["preview"] = self._make_preview(IMAGE_TOPIC, msg)
                self._last_preview_update[IMAGE_TOPIC] = now

        if self._image_request_event.is_set():
            self._latest_image_msg = msg
            self._image_capture_event.set()

    def _ensure_image_subscription(self) -> bool:
        if self._bridge is None or cv2 is None:
            return False

        with self._image_sub_lock:
            if self._image_subscriber is not None:
                return True

            msg_cls = self._image_msg_cls
            if msg_cls is None:
                available = dict(self.get_topic_names_and_types())
                if IMAGE_TOPIC not in available or not available[IMAGE_TOPIC]:
                    return False
                type_str = available[IMAGE_TOPIC][0]
                try:
                    msg_cls = get_message(type_str)
                    self._image_msg_cls = msg_cls
                    with self._lock:
                        self._topic_data[IMAGE_TOPIC]["type"] = type_str
                except Exception:
                    return False

            try:
                self._image_subscriber = self.create_subscription(
                    msg_cls,
                    IMAGE_TOPIC,
                    self._image_callback,
                    self._choose_qos(IMAGE_TOPIC),
                )
                return True
            except Exception:
                self._image_subscriber = None
                return False

    def _release_image_subscription(self) -> None:
        with self._image_sub_lock:
            if self._image_subscriber is None:
                return
            try:
                self.destroy_subscription(self._image_subscriber)
            except Exception:
                pass
            finally:
                self._image_subscriber = None

    def _make_preview(self, topic: str, msg: Any) -> str:
        if topic == "/image_raw":
            height = getattr(msg, "height", 0)
            width = getattr(msg, "width", 0)
            encoding = getattr(msg, "encoding", "unknown")
            return f"图像消息: {width}x{height}, 编码: {encoding}"

        try:
            if hasattr(msg, "data"):
                data = getattr(msg, "data")
                if isinstance(data, (list, tuple)):
                    show_len = min(len(data), 8)
                    prefix = list(data[:show_len])
                    suffix = " ..." if len(data) > show_len else ""
                    text = f"data[{len(data)}]: {prefix}{suffix}"
                else:
                    text = f"data: {data}"
            else:
                text = str(msg)
        except Exception:
            text = str(msg)

        max_len = 220
        if len(text) > max_len:
            text = text[:max_len] + "...(截断)"
        return text

    def get_topics_snapshot(self) -> List[Dict[str, Any]]:
        """!
        @brief 获取话题状态快照
        @details 返回简化的话题信息，仅包含在线状态、时间和预览
        """
        now = time.time()
        timeout_sec = 3.0
        with self._lock:
            data = []
            for topic in sorted(self._topics):
                entry = dict(self._topic_data[topic])
                last = entry["last_recv_sec"]
                entry["online"] = bool(last is not None and (now - last) <= timeout_sec)
                if last is None:
                    entry["last_recv_str"] = "-"
                else:
                    entry["last_recv_str"] = datetime.fromtimestamp(last).strftime("%H:%M:%S")
                data.append(entry)
            return data

    def capture_image_jpeg(self, timeout_sec: float = 1.8) -> Optional[bytes]:
        if not self._ensure_image_subscription():
            return None

        # 仅在用户点击刷新时短暂请求下一帧，降低持续解码开销
        self._latest_image_msg = None
        self._image_capture_event.clear()
        self._image_request_event.set()
        try:
            ok = self._image_capture_event.wait(timeout=timeout_sec)
            if not ok or self._latest_image_msg is None:
                return None
            frame = self._bridge.imgmsg_to_cv2(self._latest_image_msg, desired_encoding="bgr8")
            encode_ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
            if not encode_ok:
                return None
            return BytesIO(jpg.tobytes()).getvalue()
        except Exception:
            return None
        finally:
            self._image_request_event.clear()
            # 拍到一帧后立即释放图像订阅，避免持续高频回调占用CPU
            self._release_image_subscription()


class MonitorApp:
    """!
    @brief 封装 Flask + ROS2 运行逻辑
    @details 轻量架构：Flask依然是最轻量且兼容性最好的选择之一。
             简化了状态缓存逻辑，减少锁竞争。
    """

    def __init__(self) -> None:
        rclpy.init(args=None)
        self.monitor_node = MonitorNode(TOPICS_TO_MONITOR)
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.monitor_node)

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()
        self._status_cache_lock = threading.Lock()
        self._status_cache_expire_sec = 0.0
        self._status_cache_payload: Dict[str, Any] = {"robot_time": "-", "topics": []}

        self.app = Flask(
            __name__,
            template_folder="templates",
            static_folder="static",
        )
        self._setup_routes()

    def _spin(self) -> None:
        self.executor.spin()

    def _setup_routes(self) -> None:
        @self.app.route("/")
        def index():
            return render_template("index.html")

        @self.app.route("/api/status")
        def api_status():
            """!
            @brief 返回简化的话题状态
            @details 缓存时间延长至2秒，进一步降低轮询开销
            """
            now = time.time()
            with self._status_cache_lock:
                if now >= self._status_cache_expire_sec:
                    self._status_cache_payload = {
                        "robot_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        "topics": self.monitor_node.get_topics_snapshot(),
                    }
                    self._status_cache_expire_sec = now + 2.0  # 延长缓存，减少计算
                payload = dict(self._status_cache_payload)
            return jsonify(payload)

        @self.app.route("/api/image")
        def api_image():
            jpg_data = self.monitor_node.capture_image_jpeg()
            if jpg_data is None:
                return jsonify({"ok": False, "message": "暂未获取到 /image_raw 图像"}), 503
            return Response(jpg_data, mimetype="image/jpeg")

        @self.app.route("/api/health")
        def api_health():
            return jsonify({"ok": True})

    def run(self) -> None:
        self.app.run(host="0.0.0.0", port=8088, debug=False, threaded=True)

    def shutdown(self) -> None:
        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        self.monitor_node.destroy_node()
        rclpy.shutdown()


def main() -> None:
    web = MonitorApp()
    try:
        web.run()
    except KeyboardInterrupt:
        pass
    finally:
        web.shutdown()


if __name__ == "__main__":
    main()
