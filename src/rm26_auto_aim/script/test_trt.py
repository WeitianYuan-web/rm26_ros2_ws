#!/usr/bin/env python3
# ==============================================================================
from __future__ import annotations
import os
# 强制 CUDA 采用 blocking 同步策略，防止 CPU 在等待 GPU 时忙轮询导致占用率飙升
os.environ["CUDA_DEVICE_MAX_CONNECTIONS"] = "1"
os.environ["CUDA_MODULE_LOADING"] = "LAZY"

# @file   test_trt.py
# @brief  TensorRT engine 推理性能测试脚本
#
# 测试内容:
#   - 模型加载时间
#   - Warmup 预热
#   - 平均帧率 (FPS) 与延迟 (ms)
#   - 延迟分布 (P50 / P90 / P99)
#   - GPU 显存占用
#   - 支持图像/视频/随机噪声输入
#
# 适用平台: Jetson Xavier NX / TensorRT 8.5.x
#
# 用法:
#   python3 test_trt.py --engine ../model/yolo26n_rm_500.engine \
#                       --source ../../scrip/captured_videos/video_20260317_182846.mp4 \
#                       --iters 200
# ==============================================================================

import argparse
import time
import sys
import statistics
from pathlib import Path


# ──────────────────────────────────────────────────────────────────────────────
class C:
    OK   = "\033[92m[  OK  ]\033[0m"
    MISS = "\033[91m[ MISS ]\033[0m"
    WARN = "\033[93m[ WARN ]\033[0m"
    INFO = "\033[94m[ INFO ]\033[0m"
    ERR  = "\033[91m[ ERR  ]\033[0m"
    HEAD = "\033[1m"
    END  = "\033[0m"

def hline(title: str = "") -> None:
    bar = "─" * max(0, 54 - len(title))
    print(f"\n{C.HEAD}── {title} {bar}{C.END}")


# ──────────────────────────────────────────────────────────────────────────────
def get_gpu_mem_mb() -> tuple[int, int]:
    """@brief 获取当前 GPU 已用 / 总显存 (MB)。"""
    try:
        import torch
        used  = torch.cuda.memory_allocated(0) // (1024 * 1024)
        total = torch.cuda.get_device_properties(0).total_memory // (1024 * 1024)
        return used, total
    except Exception:
        return -1, -1


def read_tegra_stats() -> dict:
    """@brief 读取 Jetson tegrastats 快照（CPU/GPU 占用率、功耗）。"""
    import subprocess, re
    try:
        out = subprocess.check_output(
            ["tegrastats", "--interval", "500", "--count", "1"],
            stderr=subprocess.DEVNULL, timeout=3
        ).decode()
        gpu_match  = re.search(r"GR3D_FREQ\s+(\d+)%", out)
        cpu_match  = re.findall(r"(\d+)%@\d+", out)
        pow_match  = re.search(r"POM_5V_IN\s+(\d+)/", out)
        return {
            "gpu_pct"  : int(gpu_match.group(1)) if gpu_match else -1,
            "cpu_pcts" : [int(x) for x in cpu_match],
            "power_mw" : int(pow_match.group(1)) if pow_match else -1,
        }
    except Exception:
        return {}


# ──────────────────────────────────────────────────────────────────────────────
def _load_input(source_path: str | None, imgsz: int, max_frames: int):
    """
    @brief 加载测试图像或视频，并预先全部读取到内存，防止干扰推理耗时测试。
    @return list of numpy.ndarray
    """
    import numpy as np
    import cv2

    frames = []
    if source_path and Path(source_path).exists():
        path_str = str(source_path).lower()
        if path_str.endswith((".mp4", ".avi", ".mkv")):
            print(f"{C.INFO} 检测到视频文件: {source_path}")
            cap = cv2.VideoCapture(source_path)
            count = 0
            while cap.isOpened() and count < max_frames:
                ret, frame = cap.read()
                if not ret:
                    break
                frame = cv2.resize(frame, (imgsz, imgsz))
                frames.append(frame)
                count += 1
            cap.release()
            print(f"{C.OK} 已预读取 {len(frames)} 帧视频到内存 ({imgsz}×{imgsz})")
            if len(frames) == 0:
                print(f"{C.ERR} 视频读取失败或为空，改用随机噪声")
        else:
            frame = cv2.imread(source_path)
            frame = cv2.resize(frame, (imgsz, imgsz))
            frames.append(frame)
            print(f"{C.OK} 真实图像: {source_path} ({imgsz}×{imgsz})")
    else:
        if source_path:
            print(f"{C.WARN} 文件不存在: {source_path}，改用随机噪声")
        print(f"{C.INFO} 使用随机噪声图像 ({imgsz}×{imgsz})")

    if not frames:
        frames.append(np.random.randint(0, 255, (imgsz, imgsz, 3), dtype=np.uint8))

    return frames


def _bench_loop(infer_fn, frames, warmup: int, iters: int) -> tuple[list[float], object]:
    """@brief 执行预热 + 基准测试循环。"""
    n_frames = len(frames)
    
    hline(f"Warmup  ({warmup} 帧)")
    for i in range(warmup):
        f = frames[i % n_frames]
        last = infer_fn(f)
        sys.stdout.write(f"\r  预热进度: {i + 1}/{warmup}")
        sys.stdout.flush()
    print(f"\r{C.OK} Warmup 完成        ")

    hline(f"基准测试  ({iters} 帧)")
    latencies: list[float] = []
    for i in range(iters):
        f = frames[i % n_frames]
        t0 = time.perf_counter()
        last = infer_fn(f)
        latencies.append((time.perf_counter() - t0) * 1000)

        if (i + 1) % max(1, iters // 10) == 0 or i == iters - 1:
            sys.stdout.write(
                f"\r  [{i + 1:>4d}/{iters}]  "
                f"当前: {latencies[-1]:.1f} ms  ({1000.0 / latencies[-1]:.1f} FPS)"
            )
            sys.stdout.flush()
    print()
    return latencies, last


def _print_stats(latencies: list[float], total_mb: int) -> None:
    """@brief 打印延迟统计报告。"""
    avg_ms = statistics.mean(latencies)
    med_ms = statistics.median(latencies)
    min_ms = min(latencies)
    max_ms = max(latencies)
    stdev  = statistics.stdev(latencies) if len(latencies) > 1 else 0.0

    sl  = sorted(latencies)
    p50 = sl[int(len(sl) * 0.50)]
    p90 = sl[int(len(sl) * 0.90)]
    p99 = sl[min(int(len(sl) * 0.99), len(sl) - 1)]

    avg_fps = 1000.0 / avg_ms
    max_fps = 1000.0 / min_ms

    hline("性能报告")
    print(f"  {'帧率 (FPS)':<20} avg = {C.HEAD}{avg_fps:>7.2f}{C.END}   max = {max_fps:.2f}")
    print(f"  {'延迟 avg (ms)':<20} {avg_ms:>8.2f}")
    print(f"  {'延迟 median (ms)':<20} {med_ms:>8.2f}")
    print(f"  {'延迟 min (ms)':<20} {min_ms:>8.2f}")
    print(f"  {'延迟 max (ms)':<20} {max_ms:>8.2f}")
    print(f"  {'延迟 stddev (ms)':<20} {stdev:>8.2f}")
    print(f"  {'P50 (ms)':<20} {p50:>8.2f}")
    print(f"  {'P90 (ms)':<20} {p90:>8.2f}")
    print(f"  {'P99 (ms)':<20} {p99:>8.2f}")

    used_mb2, _ = get_gpu_mem_mb()
    if used_mb2 >= 0:
        print(f"  {'GPU 显存（推理中）':<20} {used_mb2} MB / {total_mb} MB")

    hline("延迟分布直方图")
    _print_histogram(latencies)

    return avg_fps, avg_ms, p99


# ──────────────────────────────────────────────────────────────────────────────
# 方案 A: 直接 TensorRT Python API (不依赖 torchvision)
# ──────────────────────────────────────────────────────────────────────────────
def run_benchmark_trt_api(
    engine_path: str,
    imgsz: int,
    warmup: int,
    iters: int,
    source_path: str | None,
) -> None:
    import numpy as np
    import tensorrt as trt
    import torch

    hline("加载 TensorRT Engine  [直接 TRT API]")
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

    t0 = time.perf_counter()
    with open(engine_path, "rb") as f:
        raw = f.read()

    import struct
    if raw[:1] == b'/' or (len(raw) > 4 and raw[4:5] == b'{'):
        meta_len = struct.unpack_from("<I", raw, 0)[0]
        trt_bytes = raw[4 + meta_len:]
    else:
        trt_bytes = raw

    runtime = trt.Runtime(TRT_LOGGER)
    engine  = runtime.deserialize_cuda_engine(trt_bytes)
    context = engine.create_execution_context()
    load_ms = (time.perf_counter() - t0) * 1000

    print(f"{C.OK} 模型加载完成，耗时 {load_ms:.0f} ms")
    
    n_bindings = engine.num_bindings
    for i in range(n_bindings):
        name  = engine.get_binding_name(i)
        shape = engine.get_binding_shape(i)
        dtype = trt.nptype(engine.get_binding_dtype(i))
        role  = "输入" if engine.binding_is_input(i) else "输出"
        print(f"  {role}[{i}] {name}: shape={list(shape)}  dtype={dtype}")

    bindings_gpu: list[torch.Tensor] = []
    for i in range(n_bindings):
        shape = engine.get_binding_shape(i)
        dtype = trt.nptype(engine.get_binding_dtype(i))
        t_dtype = torch.float16 if dtype == np.float16 else torch.float32
        buf = torch.zeros(list(shape), dtype=t_dtype, device="cuda")
        bindings_gpu.append(buf)

    bindings_ptr = [b.data_ptr() for b in bindings_gpu]

    used_mb, total_mb = get_gpu_mem_mb()

    # 提前预处理：我们将视频读取到内存，并在基准测试中包含 CPU->GPU 和预处理的时间，这最接近真实业务流。
    hline("准备测试输入")
    frames = _load_input(source_path, imgsz, max_frames=iters + warmup)

    # 用 torch.from_numpy 可大幅加速预处理
    def preprocess(img) -> torch.Tensor:
        # HWC (uint8) -> CHW (float16/32) on GPU
        x = torch.from_numpy(img).cuda(non_blocking=True)
        x = x.float() / 255.0
        x = x.permute(2, 0, 1).contiguous()
        x = x.unsqueeze(0).half() # 假设我们用了 fp16 编译
        return x

    def infer(_frame) -> np.ndarray:
        """单次推理：包含数据预处理、拷贝、GPU 推理和结果回传"""
        bindings_gpu[0].copy_(preprocess(_frame))
        context.execute_v2(bindings_ptr)
        torch.cuda.synchronize()  # 必须同步，否则只会测出下发任务的时间
        return bindings_gpu[-1].cpu().numpy()

    latencies, last_out = _bench_loop(infer, frames, warmup, iters)
    avg_fps, avg_ms, p99 = _print_stats(latencies, total_mb)

    hline("输出张量示例（最后一帧，前5个检测）")
    if last_out.ndim == 3:              # (1, N, 6) → (N, 6)
        dets = last_out[0]
    elif last_out.ndim == 2:
        dets = last_out
    else:
        dets = last_out.reshape(-1, last_out.shape[-1])

    valid = dets[dets[:, 4] > 0.25] if dets.shape[1] >= 5 else dets
    print(f"  conf>0.25 的目标数: {len(valid)}")
    for j, d in enumerate(valid[:5]):
        print(f"  [{j}] xyxy=[{d[0]:.0f},{d[1]:.0f},{d[2]:.0f},{d[3]:.0f}]"
              f"  conf={d[4]:.3f}"
              + (f"  cls={int(d[5])}" if len(d) > 5 else ""))

    stats = read_tegra_stats()
    if stats:
        hline("Jetson 系统状态")
        if stats.get("gpu_pct", -1) >= 0:
            print(f"  GPU 利用率  : {stats['gpu_pct']}%")
        if stats.get("cpu_pcts"):
            print(f"  CPU 均值    : {statistics.mean(stats['cpu_pcts']):.0f}%")
        if stats.get("power_mw", -1) >= 0:
            print(f"  系统功耗    : {stats['power_mw'] / 1000:.2f} W")

    hline("综合评估")
    _print_assessment(avg_fps, avg_ms, p99)


# ──────────────────────────────────────────────────────────────────────────────
# 方案 B: Ultralytics YOLO 高层 API（依赖 torchvision NMS）
# ──────────────────────────────────────────────────────────────────────────────
def run_benchmark_ultralytics(
    engine_path: str,
    imgsz: int,
    warmup: int,
    iters: int,
    source_path: str | None,
) -> None:
    hline("加载 TensorRT Engine  [Ultralytics API]")
    from ultralytics import YOLO

    t0 = time.perf_counter()
    model = YOLO(engine_path, task="detect")
    load_ms = (time.perf_counter() - t0) * 1000
    print(f"{C.OK} 模型加载完成，耗时 {load_ms:.0f} ms")

    hline("准备测试输入")
    frames = _load_input(source_path, imgsz, max_frames=iters + warmup)
    used_mb, total_mb = get_gpu_mem_mb()

    def infer(_frame):
        return model.predict(_frame, device=0, verbose=False, imgsz=imgsz)

    latencies, results = _bench_loop(infer, frames, warmup, iters)
    avg_fps, avg_ms, p99 = _print_stats(latencies, total_mb)

    hline("检测结果示例（最后一帧）")
    result = results[0]
    boxes  = result.boxes
    if boxes is not None and len(boxes) > 0:
        names = result.names
        print(f"  检测到目标数: {len(boxes)}")
        for j, box in enumerate(boxes[:5]):
            cls_id = int(box.cls[0])
            conf   = float(box.conf[0])
            xyxy   = box.xyxy[0].tolist()
            label  = names.get(cls_id, str(cls_id))
            print(f"  [{j}] {label:<12s}  conf={conf:.3f}  "
                  f"xyxy=[{xyxy[0]:.0f},{xyxy[1]:.0f},{xyxy[2]:.0f},{xyxy[3]:.0f}]")
    else:
        print(f"  未检测到目标")

    hline("综合评估")
    _print_assessment(avg_fps, avg_ms, p99)


def run_benchmark(
    engine_path: str,
    imgsz: int,
    warmup: int,
    iters: int,
    source_path: str | None,
    mode: str = "auto",
) -> None:
    if mode == "ultralytics":
        run_benchmark_ultralytics(engine_path, imgsz, warmup, iters, source_path)
        return

    if mode == "auto":
        try:
            import tensorrt  # noqa: F401
            mode = "trt"
        except ImportError:
            mode = "ultralytics"

    if mode == "trt":
        run_benchmark_trt_api(engine_path, imgsz, warmup, iters, source_path)
    else:
        run_benchmark_ultralytics(engine_path, imgsz, warmup, iters, source_path)


def _print_histogram(latencies: list[float], bins: int = 10, width: int = 40) -> None:
    if not latencies:
        return
    lo, hi = min(latencies), max(latencies)
    if lo == hi:
        print(f"  所有延迟均为 {lo:.2f} ms")
        return
    step = (hi - lo) / bins
    counts = [0] * bins
    for v in latencies:
        idx = min(int((v - lo) / step), bins - 1)
        counts[idx] += 1
    max_cnt = max(counts) or 1
    for i, cnt in enumerate(counts):
        left  = lo + i * step
        right = left + step
        bar   = "█" * int(cnt / max_cnt * width)
        print(f"  {left:>6.1f}-{right:<6.1f} ms |{bar:<{width}}| {cnt}")


def _print_assessment(avg_fps: float, avg_ms: float, p99_ms: float) -> None:
    if avg_fps >= 60:
        grade = f"{C.OK} 优秀 (≥60 FPS)"
        tip   = "TRT FP16 性能充裕，可胜任实时检测"
    elif avg_fps >= 30:
        grade = f"\033[92m[ GOOD ]\033[0m 良好 (≥30 FPS)"
        tip   = "满足实时需求"
    elif avg_fps >= 15:
        grade = f"{C.WARN} 一般 (≥15 FPS)"
        tip   = "勉强实时"
    else:
        grade = f"{C.ERR} 偏慢 (<15 FPS)"
        tip   = "建议检查性能模式"

    print(f"  评级    : {grade}")
    print(f"  平均帧率: {avg_fps:.1f} FPS  ({avg_ms:.1f} ms/frame)")
    print(f"  P99 延迟: {p99_ms:.1f} ms")
    print(f"  建议    : {tip}")


# ──────────────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    script_dir   = Path(__file__).resolve().parent
    default_eng  = str(script_dir / "../model/yolo26n_rm_500.engine")

    parser = argparse.ArgumentParser(
        description="TensorRT engine 推理性能测试  (Jetson Xavier NX)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--engine", "-e",
        type=str,
        default=default_eng,
        help="TensorRT .engine 文件路径",
    )
    parser.add_argument(
        "--source", "-s",
        type=str,
        default=None,
        help="测试图像或视频路径（如不指定则用随机噪声）",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="推理图像边长 (默认: 640)",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=10,
        help="预热帧数 (默认: 10)",
    )
    parser.add_argument(
        "--iters",
        type=int,
        default=100,
        help="基准测试帧数 (默认: 100)",
    )
    parser.add_argument(
        "--mode",
        type=str,
        default="auto",
        choices=["auto", "trt", "ultralytics"],
        help="推理后端: auto / trt / ultralytics",
    )
    return parser.parse_args()


def main() -> None:
    print(f"\n{C.HEAD}{'═' * 56}")
    print("  TensorRT Engine 推理性能测试")
    print(f"  Jetson Xavier NX  |  CUDA 11.4  |  TensorRT 8.5.x")
    print(f"{'═' * 56}{C.END}")

    args = parse_args()

    engine_path = str(Path(args.engine).resolve())
    if not Path(engine_path).exists():
        print(f"{C.ERR} engine 文件不存在: {engine_path}")
        sys.exit(1)

    run_benchmark(
        engine_path = engine_path,
        imgsz       = args.imgsz,
        warmup      = args.warmup,
        iters       = args.iters,
        source_path = args.source,
        mode        = args.mode,
    )


if __name__ == "__main__":
    main()
