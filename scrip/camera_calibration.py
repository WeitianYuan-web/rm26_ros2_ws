#!/usr/bin/env python3
"""
相机标定脚本 — 支持 ROS2 实时采集 和 离线图片 两种模式
自动检测是否有显示器，无显示器时自动切换为「无头自动采集」模式。

模式 1: ROS2 实时采集 (默认)
  有显示器:
    显示实时画面，按 'c' 采集，按 'q' 标定。
  无显示器 (SSH / Cursor Remote):
    自动检测棋盘格并采集，终端显示进度，按 Enter 提前结束。

    python3 camera_calibration.py --rows 6 --cols 9 --square 25.0

模式 2: 离线图片标定
    python3 camera_calibration.py --offline --image-dir ./calib_images \
        --rows 6 --cols 9 --square 25.0

输出:
    1. camera_calibration.yaml  — YAML 格式标定结果
    2. 终端打印 hk_camera_node ROS2 参数格式，可直接复制到 launch 文件
    3. 标定图片保存在 ./calib_images/ 目录 (ROS2 模式)

依赖:
    pip3 install opencv-python numpy pyyaml
    (ROS2 模式额外需要: rclpy, sensor_msgs, cv_bridge)
"""

import argparse
import os
import sys
import time
import glob
import select
import numpy as np
import cv2
import yaml


# ==================== 显示器检测 ====================

def check_display() -> bool:
    """
    检测当前环境是否有可用的图形显示器。
    使用 xdpyinfo 安全探测，不触发 GTK 致命错误。

    依次尝试: 已有 DISPLAY -> :0 -> :1 -> 判定为无头。

    Returns:
        True 表示有可用显示器, False 表示无头模式
    """
    import subprocess

    display = os.environ.get('DISPLAY', '')

    # 已经设置了 DISPLAY，验证
    if display and _test_display_safe(display):
        return True

    # 尝试常见的本地显示
    for d in [':0', ':1']:
        if _test_display_safe(d):
            os.environ['DISPLAY'] = d
            print(f"  自动检测到显示器: DISPLAY={d}")
            return True

    return False


def _test_display_safe(display: str) -> bool:
    """
    安全测试指定 DISPLAY 是否可用。
    使用 xdpyinfo 子进程探测，避免 GTK 致命错误。
    """
    import subprocess
    env = os.environ.copy()
    env['DISPLAY'] = display
    try:
        ret = subprocess.run(
            ['xdpyinfo'],
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=3,
        )
        return ret.returncode == 0
    except FileNotFoundError:
        # xdpyinfo 未安装，退化为检查 X11 socket
        socket_path = f"/tmp/.X11-unix/X{display.split(':')[-1].split('.')[0]}"
        return os.path.exists(socket_path)
    except Exception:
        return False


def stdin_has_input() -> bool:
    """非阻塞检测终端是否有 Enter 输入"""
    try:
        return select.select([sys.stdin], [], [], 0.0)[0] != []
    except Exception:
        return False


# ==================== 标定器 ====================

class ChessboardCalibrator:
    """棋盘格相机标定器"""

    def __init__(self, rows: int, cols: int, square_size: float):
        """
        Args:
            rows: 内角点行数
            cols: 内角点列数
            square_size: 方格边长 (mm)
        """
        self.rows = rows
        self.cols = cols
        self.square_size = square_size
        self.pattern_size = (cols, rows)

        # 3D 物体点 (棋盘格角点在世界坐标系中的位置)
        self.objp = np.zeros((rows * cols, 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
        self.objp *= square_size  # 单位: mm

        # 存储标定数据
        self.obj_points = []   # 3D 点
        self.img_points = []   # 2D 点
        self.image_size = None
        self.captured_count = 0

        # 用于去重：上次采集的角点中心位置和完整角点坐标
        self.last_center = None
        self.last_corners_flat = None

        # 角点检测参数
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                         30, 0.001)
        self.find_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH +
                           cv2.CALIB_CB_NORMALIZE_IMAGE +
                           cv2.CALIB_CB_FAST_CHECK)

    def detect_corners(self, img: np.ndarray):
        """
        在图像中检测棋盘格角点

        Returns:
            (success, corners, vis_img)
        """
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.image_size is None:
            self.image_size = (gray.shape[1], gray.shape[0])

        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size,
                                                  self.find_flags)
        vis_img = img.copy()

        if ret:
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                        self.criteria)
            cv2.drawChessboardCorners(vis_img, self.pattern_size, corners, ret)
            cv2.rectangle(vis_img, (0, 0),
                          (vis_img.shape[1] - 1, vis_img.shape[0] - 1),
                          (0, 255, 0), 4)
        else:
            cv2.rectangle(vis_img, (0, 0),
                          (vis_img.shape[1] - 1, vis_img.shape[0] - 1),
                          (0, 0, 255), 4)

        return ret, corners, vis_img

    def is_sufficiently_different(self, corners: np.ndarray,
                                  min_move_px: float = 30.0) -> bool:
        """
        判断当前角点位置/姿态是否与上一次采集有足够差异。
        同时比较角点中心位移和角点分布形状变化（检测旋转/倾斜），
        任一变化足够大即视为有效新样本。

        Args:
            corners: 当前角点
            min_move_px: 角点中心最小移动像素

        Returns:
            True 表示差异足够大，应当采集
        """
        pts = corners.reshape(-1, 2)
        center = np.mean(pts, axis=0)

        if self.last_center is None:
            self.last_corners_flat = pts.copy()
            return True

        # 条件 1: 中心位移足够大
        center_dist = np.linalg.norm(center - self.last_center)
        if center_dist >= min_move_px:
            return True

        # 条件 2: 角点分布形状变化（检测旋转/倾斜变化）
        # 比较所有角点相对中心的偏移差异
        if self.last_corners_flat is not None:
            curr_rel = pts - center
            last_rel = self.last_corners_flat - self.last_center
            shape_diff = np.mean(np.linalg.norm(curr_rel - last_rel, axis=1))
            if shape_diff >= min_move_px * 0.5:
                return True

        return False

    def add_sample(self, corners: np.ndarray):
        """添加一组角点作为标定样本"""
        self.obj_points.append(self.objp.copy())
        self.img_points.append(corners)
        self.captured_count += 1
        pts = corners.reshape(-1, 2)
        self.last_center = np.mean(pts, axis=0)
        self.last_corners_flat = pts.copy()

    def calibrate(self):
        """执行标定"""
        if self.captured_count < 3:
            print(f"错误: 至少需要 3 帧标定图片，当前只有 {self.captured_count} 帧")
            return None

        print(f"\n正在标定 (使用 {self.captured_count} 帧图片)...")
        start_time = time.time()

        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points, self.img_points, self.image_size, None, None
        )

        elapsed = time.time() - start_time
        print(f"标定完成! 耗时: {elapsed:.1f}s")

        total_error = 0
        per_image_errors = []
        for i in range(len(self.obj_points)):
            img_points_proj, _ = cv2.projectPoints(
                self.obj_points[i], rvecs[i], tvecs[i],
                camera_matrix, dist_coeffs
            )
            error = cv2.norm(self.img_points[i], img_points_proj,
                             cv2.NORM_L2) / len(img_points_proj)
            per_image_errors.append(error)
            total_error += error

        mean_error = total_error / len(self.obj_points)

        return {
            'camera_matrix': camera_matrix,
            'dist_coeffs': dist_coeffs,
            'rvecs': rvecs,
            'tvecs': tvecs,
            'rms_error': rms,
            'mean_reproj_error': mean_error,
            'per_image_errors': per_image_errors,
            'image_size': self.image_size,
            'num_images': self.captured_count,
        }


# ==================== 结果输出 ====================

def save_calibration(result: dict, output_path: str):
    """保存标定结果到 YAML 文件"""
    K = result['camera_matrix']
    D = result['dist_coeffs']

    data = {
        'image_width': result['image_size'][0],
        'image_height': result['image_size'][1],
        'camera_matrix': {
            'rows': 3, 'cols': 3,
            'data': K.flatten().tolist(),
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1, 'cols': 5,
            'data': D.flatten().tolist(),
        },
        'fx': float(K[0, 0]),
        'fy': float(K[1, 1]),
        'cx': float(K[0, 2]),
        'cy': float(K[1, 2]),
        'rms_error': float(result['rms_error']),
        'mean_reproj_error': float(result['mean_reproj_error']),
        'num_images': result['num_images'],
    }

    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False)

    print(f"\n标定结果已保存: {output_path}")


def print_ros2_params(result: dict):
    """打印可直接用于 hk_camera_node 的 ROS2 参数"""
    K = result['camera_matrix']
    D = result['dist_coeffs'].flatten()

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    print("\n" + "=" * 65)
    print("  hk_camera_node ROS2 参数 (复制到 launch 文件或命令行)")
    print("=" * 65)

    print(f"""
# Launch 文件参数格式:
Node(
    package='rm_auto_aim',
    executable='hk_camera_node',
    parameters=[{{
        'camera_info.fx': {fx:.6f},
        'camera_info.fy': {fy:.6f},
        'camera_info.cx': {cx:.6f},
        'camera_info.cy': {cy:.6f},
        'camera_info.distortion': {D.tolist()},
    }}],
)

# 命令行参数格式:
ros2 run rm_auto_aim hk_camera_node \\
    --ros-args \\
    -p camera_info.fx:={fx:.6f} \\
    -p camera_info.fy:={fy:.6f} \\
    -p camera_info.cx:={cx:.6f} \\
    -p camera_info.cy:={cy:.6f}""")
    print("=" * 65)


def print_result_summary(result: dict):
    """打印标定结果摘要"""
    K = result['camera_matrix']
    D = result['dist_coeffs'].flatten()

    print("\n" + "=" * 50)
    print("            相机标定结果")
    print("=" * 50)
    print(f"  图像尺寸:       {result['image_size'][0]} x {result['image_size'][1]}")
    print(f"  使用图片数:     {result['num_images']}")
    print(f"  RMS 重投影误差: {result['rms_error']:.4f} px")
    print(f"  平均重投影误差: {result['mean_reproj_error']:.4f} px")
    print(f"")
    print(f"  相机内参矩阵 K:")
    print(f"    fx = {K[0, 0]:.4f}")
    print(f"    fy = {K[1, 1]:.4f}")
    print(f"    cx = {K[0, 2]:.4f}")
    print(f"    cy = {K[1, 2]:.4f}")
    print(f"")
    print(f"  畸变系数 D (plumb_bob):")
    print(f"    k1 = {D[0]:.6f}")
    print(f"    k2 = {D[1]:.6f}")
    print(f"    p1 = {D[2]:.6f}")
    print(f"    p2 = {D[3]:.6f}")
    print(f"    k3 = {D[4]:.6f}")
    print("=" * 50)

    rms = result['rms_error']
    if rms < 0.3:
        quality = "优秀"
    elif rms < 0.5:
        quality = "良好"
    elif rms < 1.0:
        quality = "一般 (建议重新标定)"
    else:
        quality = "较差 (请重新标定，注意标定板平整度和拍摄角度多样性)"
    print(f"\n  标定质量: {quality} (RMS = {rms:.4f})")

    errors = result['per_image_errors']
    if max(errors) > 2 * np.mean(errors):
        bad_frames = [i for i, e in enumerate(errors)
                      if e > 2 * np.mean(errors)]
        print(f"  注意: 第 {bad_frames} 帧重投影误差偏高，"
              f"可考虑移除后重新标定")


# ==================== ROS2 实时模式 (有显示器) ====================

def read_terminal_key() -> str:
    """
    非阻塞读取终端按键。
    返回用户输入的第一个字符（小写），无输入则返回空串。
    """
    if not stdin_has_input():
        return ''
    line = sys.stdin.readline().strip().lower()
    return line[0] if line else ''


def run_ros2_gui_mode(calibrator, node, save_dir, min_frames):
    """ROS2 实时 GUI 模式 — 有显示器时使用，同时监听终端输入"""
    import rclpy

    print("\n  模式: GUI 交互采集")
    print("  操作说明 (OpenCV 窗口按键 或 终端输入均可):")
    print("    [c]+Enter  采集当前帧 (检测到角点时)")
    print("    [q]+Enter  结束采集，开始标定")
    print("    [x]+Enter  退出不标定")
    print("=" * 50)

    cv2.namedWindow("Camera Calibration", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Calibration", 640, 480)

    # 预创建等待画面
    wait_img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(wait_img, "Waiting for /image_raw ...",
                (80, 220), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 255), 2)
    cv2.putText(wait_img, "[q]+Enter = quit",
                (200, 280), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (180, 180, 180), 1)

    corner_detected = False  # 当前帧是否检测到角点

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)

            if node.latest_img is None:
                # 无图像时显示等待画面，保持窗口刷新
                cv2.imshow("Camera Calibration", wait_img)
                cv_key = cv2.waitKey(30) & 0xFF
                term_key = read_terminal_key()
                if cv_key == 27 or term_key == 'x':
                    print("用户取消")
                    return None
                if cv_key == ord('q') or term_key == 'q':
                    print("用户取消")
                    return None
                continue

            img = node.latest_img.copy()
            ret, corners, vis_img = calibrator.detect_corners(img)
            corner_detected = ret

            status = (f"Captured: {calibrator.captured_count}/{min_frames} | "
                      f"Corners: {'YES' if ret else 'NO'} | "
                      f"[c]Capture [q]Calibrate")
            cv2.putText(vis_img, status, (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

            cv2.imshow("Camera Calibration", vis_img)

            # 同时监听 OpenCV 窗口按键 和 终端输入
            cv_key = cv2.waitKey(1) & 0xFF
            term_key = read_terminal_key()

            key_c = (cv_key == ord('c') or term_key == 'c')
            key_q = (cv_key == ord('q') or term_key == 'q')
            key_esc = (cv_key == 27 or term_key == 'x')

            if key_c and corner_detected:
                calibrator.add_sample(corners)
                fname = os.path.join(
                    save_dir,
                    f"calib_{calibrator.captured_count:03d}.png")
                cv2.imwrite(fname, img)
                print(f"  [+] 已采集第 {calibrator.captured_count} 帧: {fname}")
            elif key_c and not corner_detected:
                print(f"  [!] 当前未检测到角点，请调整标定板位置")

            elif key_q:
                if calibrator.captured_count >= 3:
                    print(f"\n采集完成，共 {calibrator.captured_count} 帧")
                    break
                else:
                    print(f"  至少需要 3 帧，当前 {calibrator.captured_count} 帧")

            elif key_esc:
                print("用户取消")
                return None

    except KeyboardInterrupt:
        print("\n用户中断 (Ctrl+C)")
        if calibrator.captured_count < 3:
            return None

    finally:
        cv2.destroyAllWindows()

    return calibrator.calibrate()


# ==================== ROS2 实时模式 (无头) ====================

def run_ros2_headless_mode(calibrator, node, save_dir, min_frames,
                           capture_interval):
    """
    ROS2 无头自动采集模式 — 无显示器时使用。
    自动检测棋盘格角点，位姿变化足够大时自动采集。
    """
    import rclpy

    print("\n  模式: 无头自动采集 (无显示器)")
    print(f"  采集间隔: {capture_interval:.1f}s")
    print(f"  自动采集: 检测到角点且位姿变化足够大时自动保存")
    print(f"  按 Enter 提前结束采集并开始标定")
    print("=" * 50)

    last_capture_time = 0
    no_corner_count = 0
    waiting_for_image = True

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)

            # 检查终端输入 (Enter 键)
            if stdin_has_input():
                sys.stdin.readline()
                if calibrator.captured_count >= 3:
                    print(f"\n用户手动结束，共 {calibrator.captured_count} 帧")
                    break
                else:
                    print(f"  至少需要 3 帧，当前 {calibrator.captured_count} 帧，继续采集...")

            if node.latest_img is None:
                continue

            if waiting_for_image:
                waiting_for_image = False
                print("  收到图像，开始检测棋盘格...")
                print("  请手持标定板在相机前缓慢移动，变换角度和距离\n")

            img = node.latest_img.copy()
            now = time.time()

            # 采集间隔未到，跳过检测
            if now - last_capture_time < capture_interval:
                continue

            ret, corners, _ = calibrator.detect_corners(img)

            if ret:
                no_corner_count = 0

                if calibrator.is_sufficiently_different(corners):
                    calibrator.add_sample(corners)
                    fname = os.path.join(
                        save_dir,
                        f"calib_{calibrator.captured_count:03d}.png")
                    cv2.imwrite(fname, img)
                    last_capture_time = now

                    progress = calibrator.captured_count
                    bar_len = 20
                    filled = int(bar_len * min(progress, min_frames) / min_frames)
                    bar = '█' * filled + '░' * (bar_len - filled)
                    print(f"  [+] 采集 {progress:2d}/{min_frames} "
                          f"[{bar}] -> {os.path.basename(fname)}")

                    if progress >= min_frames:
                        print(f"\n已达到目标帧数 {min_frames}，自动开始标定")
                        break
                else:
                    # 位姿太接近，提示用户移动
                    if now - last_capture_time > capture_interval * 3:
                        print("  [~] 检测到角点但位姿变化不够，请移动标定板...")
                        last_capture_time = now - capture_interval + 0.5
            else:
                no_corner_count += 1
                if no_corner_count == 50:
                    print("  [?] 持续未检测到角点，请确认:")
                    print("      - 标定板在画面内")
                    print("      - 光照充足，无严重反光")
                    print("      - 标定板尽量保持平整")
                    no_corner_count = 0

    except KeyboardInterrupt:
        print("\n用户中断 (Ctrl+C)")
        if calibrator.captured_count < 3:
            return None

    return calibrator.calibrate()


# ==================== ROS2 入口 ====================

def run_ros2_mode(calibrator: ChessboardCalibrator, save_dir: str,
                  min_frames: int, capture_interval: float,
                  has_display: bool):
    """ROS2 实时采集模式"""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError:
        print("错误: ROS2 模式需要 rclpy, sensor_msgs, cv_bridge")
        print("请确认已 source ROS2 环境: source /opt/ros/foxy/setup.bash")
        sys.exit(1)

    os.makedirs(save_dir, exist_ok=True)

    class CalibCollector(Node):
        """标定图像采集节点"""
        def __init__(self):
            super().__init__('camera_calibrator')
            self.bridge = CvBridge()
            self.latest_img = None
            # 使用 SensorDataQoS (BEST_EFFORT) 匹配 hk_camera_node 的发布 QoS
            self.sub = self.create_subscription(
                Image, '/image_raw', self.img_callback,
                qos_profile_sensor_data)
            self.get_logger().info("等待 /image_raw 话题...")

        def img_callback(self, msg):
            try:
                self.latest_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            except Exception as e:
                self.get_logger().warn(f"图像转换失败: {e}")

    rclpy.init()
    node = CalibCollector()

    print("\n" + "=" * 50)
    print("  ROS2 实时标定")
    print("=" * 50)
    print(f"  棋盘格: {calibrator.cols}x{calibrator.rows} 内角点")
    print(f"  方格边长: {calibrator.square_size} mm")
    print(f"  目标帧数: {min_frames}")
    print(f"  图片保存: {os.path.abspath(save_dir)}/")

    try:
        if has_display:
            result = run_ros2_gui_mode(calibrator, node, save_dir, min_frames)
        else:
            result = run_ros2_headless_mode(calibrator, node, save_dir,
                                             min_frames, capture_interval)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return result


# ==================== 离线图片模式 ====================

def run_offline_mode(calibrator: ChessboardCalibrator, image_dir: str,
                     has_display: bool):
    """离线图片标定模式"""
    extensions = ['*.png', '*.jpg', '*.jpeg', '*.bmp']
    image_files = []
    for ext in extensions:
        image_files.extend(glob.glob(os.path.join(image_dir, ext)))
    image_files.sort()

    if not image_files:
        print(f"错误: 在 {image_dir} 中未找到图片文件")
        sys.exit(1)

    print(f"\n找到 {len(image_files)} 张图片，开始检测角点...")

    if has_display:
        cv2.namedWindow("Offline Calibration", cv2.WINDOW_NORMAL)

    for i, fpath in enumerate(image_files):
        img = cv2.imread(fpath)
        if img is None:
            print(f"  [-] 跳过 (无法读取): {fpath}")
            continue

        ret, corners, vis_img = calibrator.detect_corners(img)

        if ret:
            calibrator.add_sample(corners)
            print(f"  [+] ({calibrator.captured_count}/{len(image_files)}) "
                  f"检测到角点: {os.path.basename(fpath)}")
        else:
            print(f"  [-] ({i + 1}/{len(image_files)}) "
                  f"未检测到角点: {os.path.basename(fpath)}")

        if has_display:
            cv2.imshow("Offline Calibration", vis_img)
            if cv2.waitKey(200) & 0xFF == 27:
                break

    if has_display:
        cv2.destroyAllWindows()

    if calibrator.captured_count < 3:
        print(f"\n错误: 仅检测到 {calibrator.captured_count} 帧有效角点，不足以标定")
        return None

    return calibrator.calibrate()


# ==================== 主函数 ====================

def main():
    parser = argparse.ArgumentParser(
        description="相机标定工具 (ROS2 实时 / 离线图片, 支持有头/无头模式)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # 棋盘格参数
    parser.add_argument("--rows", type=int, default=6,
                        help="内角点行数 (默认: 6)")
    parser.add_argument("--cols", type=int, default=9,
                        help="内角点列数 (默认: 9)")
    parser.add_argument("--square", type=float, default=25.0,
                        help="方格边长 mm (默认: 25.0)")

    # 模式选择
    parser.add_argument("--offline", action="store_true",
                        help="使用离线图片模式 (默认: ROS2 实时模式)")
    parser.add_argument("--image-dir", type=str, default="./calib_images",
                        help="标定图片目录 (离线模式读取 / 实时模式保存)")
    parser.add_argument("--headless", action="store_true",
                        help="强制无头模式 (不尝试打开窗口)")

    # 采集参数
    parser.add_argument("--min-frames", type=int, default=15,
                        help="目标采集帧数 (默认: 15)")
    parser.add_argument("--interval", type=float, default=2.0,
                        help="无头模式自动采集间隔秒数 (默认: 2.0)")

    # 输出参数
    parser.add_argument("--output", type=str,
                        default="camera_calibration.yaml",
                        help="标定结果输出文件 (默认: camera_calibration.yaml)")

    args = parser.parse_args()

    # 检测显示器
    if args.headless:
        has_display = False
        print("[信息] 已指定 --headless，使用无头模式")
    else:
        print("[信息] 正在检测图形显示器...")
        has_display = check_display()
        if has_display:
            print("[信息] 显示器可用，将使用 GUI 交互模式")
        else:
            print("[信息] 未检测到显示器，将使用无头自动采集模式")
            print("[提示] 如果有显示器但检测失败，可尝试:")
            print("       export DISPLAY=:0  (或 :1)")
            print("       然后重新运行本脚本\n")

    # 创建标定器
    calibrator = ChessboardCalibrator(args.rows, args.cols, args.square)

    # 运行标定
    if args.offline:
        result = run_offline_mode(calibrator, args.image_dir, has_display)
    else:
        result = run_ros2_mode(calibrator, args.image_dir, args.min_frames,
                               args.interval, has_display)

    if result is None:
        print("标定失败或已取消")
        sys.exit(1)

    # 输出结果
    print_result_summary(result)
    save_calibration(result, args.output)
    print_ros2_params(result)


if __name__ == "__main__":
    main()
