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


class MonitorNode(Node):
    """! @brief ROS2 监控节点，负责订阅并缓存话题状态 """

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
        self._bridge = CvBridge() if CvBridge is not None else None

        for topic in self._topics:
            self._topic_data[topic] = self._init_topic_entry(topic)
            self._last_preview_update[topic] = 0.0
            self._last_sample_update[topic] = 0.0

        self._discovery_timer = self.create_timer(2.0, self._discover_and_subscribe)

    def _init_topic_entry(self, topic: str) -> Dict[str, Any]:
        return {
            "topic": topic,
            "type": "unknown",
            "online": False,
            "last_recv_sec": None,
            "last_recv_str": "-",
            "count": 0,
            "hz": 0.0,
            "preview": "等待消息...",
        }

    def _discover_and_subscribe(self) -> None:
        available = dict(self.get_topic_names_and_types())
        for topic in self._topics:
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
        def _callback(msg) -> None:
            now = time.time()
            sample_interval = 0.10 if topic in HEAVY_TOPICS else 0.02
            if (now - self._last_sample_update[topic]) < sample_interval:
                if topic == "/image_raw" and self._image_request_event.is_set():
                    self._latest_image_msg = msg
                    self._image_capture_event.set()
                return

            preview_interval = 1.2 if topic in HEAVY_TOPICS else 0.4
            update_preview = (now - self._last_preview_update[topic]) >= preview_interval

            with self._lock:
                entry = self._topic_data[topic]
                entry["online"] = True
                entry["last_recv_str"] = datetime.fromtimestamp(now).strftime("%H:%M:%S")
                entry["count"] += 1
                last_recv_sec = entry["last_recv_sec"]
                entry["last_recv_sec"] = now

                if last_recv_sec is not None:
                    dt = now - last_recv_sec
                    if dt > 1e-6:
                        inst_hz = 1.0 / dt
                        entry["hz"] = round((entry["hz"] * 0.7) + (inst_hz * 0.3), 2)

                if update_preview:
                    entry["preview"] = self._make_preview(topic, msg)
                    self._last_preview_update[topic] = now
                self._last_sample_update[topic] = now

            if topic == "/image_raw" and self._image_request_event.is_set():
                self._latest_image_msg = msg
                self._image_capture_event.set()

        return _callback

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
        now = time.time()
        timeout_sec = 3.0
        with self._lock:
            data = []
            for topic in sorted(self._topics):
                entry = dict(self._topic_data[topic])
                last = entry["last_recv_sec"]
                entry["online"] = bool(last is not None and (now - last) <= timeout_sec)
                data.append(entry)
            return data

    def capture_image_jpeg(self, timeout_sec: float = 1.8) -> Optional[bytes]:
        if self._bridge is None or cv2 is None:
            return None

        # 仅在用户点击刷新时短暂请求下一帧，降低持续解码开销
        self._latest_image_msg = None
        self._image_capture_event.clear()
        self._image_request_event.set()
        ok = self._image_capture_event.wait(timeout=timeout_sec)
        self._image_request_event.clear()
        if not ok or self._latest_image_msg is None:
            return None

        try:
            frame = self._bridge.imgmsg_to_cv2(self._latest_image_msg, desired_encoding="bgr8")
            encode_ok, jpg = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 78])
            if not encode_ok:
                return None
            return BytesIO(jpg.tobytes()).getvalue()
        except Exception:
            return None


class MonitorApp:
    """! @brief 封装 Flask + ROS2 运行逻辑 """

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
            now = time.time()
            with self._status_cache_lock:
                if now >= self._status_cache_expire_sec:
                    self._status_cache_payload = {
                        "robot_time": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                        "topics": self.monitor_node.get_topics_snapshot(),
                    }
                    self._status_cache_expire_sec = now + 0.35
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
