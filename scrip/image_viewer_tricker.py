#!/usr/bin/env python3
"""
自动瞄准可视化工具 — 订阅图像、检测结果、追踪结果并叠加显示

订阅话题:
    /image_raw            (sensor_msgs/Image)           — 原始图像
    /detector/armors      (auto_aim_interfaces/Armors)  — 检测到的装甲板
    /tracker/target       (auto_aim_interfaces/Target)  — 追踪目标

用法:
    # 默认使用
    python3 image_viewer.py

    # 指定图像话题
    python3 image_viewer.py --topic /image_raw

    # 不显示追踪信息 (只看检测)
    python3 image_viewer.py --no-tracker

快捷键:
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
"""

import argparse
import math
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np

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

        # 半透明背景
        overlay = display.copy()
        cv2.rectangle(
            overlay,
            (panel_x, panel_y),
            (panel_x + panel_w, panel_y + panel_h),
            (0, 0, 0), -1,
        )
        cv2.addWeighted(overlay, 0.6, display, 0.4, 0, display)

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
        @brief 主循环：处理 ROS2 回调 + OpenCV 窗口显示
        """
        if not ensure_display():
            self.get_logger().error(
                "未检测到可用的显示器。"
                "请确保有物理显示器连接，或通过 X11 转发 (ssh -X) 连接。"
                "也可手动设置: export DISPLAY=:0"
            )
            return

        window_name = f"Auto Aim Viewer - {self.topic_name_}"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)

        # 等待画面
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

        try:
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.01)
                self.calculate_fps()

                if self.current_frame_ is not None:
                    display = self.draw_overlay(self.current_frame_)

                    # 第一帧到达时自适应窗口
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
