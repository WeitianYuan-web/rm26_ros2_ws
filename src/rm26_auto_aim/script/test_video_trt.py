#!/usr/bin/env python3
from __future__ import annotations

import argparse
import struct
import time
from pathlib import Path

import cv2
import numpy as np
import tensorrt as trt
import torch


def parse_args() -> argparse.Namespace:
    """@brief 解析命令行参数。"""
    parser = argparse.ArgumentParser(description="TensorRT 视频检测并保存叠加框输出")
    parser.add_argument(
        "--engine",
        type=str,
        default="/home/linkerhand/RMUL2026/rm26_ros2_ws/src/rm26_auto_aim/model/yolo26n_rm_500_n.engine",
        help="TensorRT engine 文件路径",
    )
    parser.add_argument(
        "--input",
        type=str,
        required=True,
        help="输入视频路径",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="",
        help="输出视频路径；为空时自动在输入视频名后追加 _detected_trt.mp4",
    )
    parser.add_argument("--imgsz", type=int, default=640, help="推理输入尺寸（默认640）")
    parser.add_argument("--conf", type=float, default=0.25, help="置信度阈值（默认0.25）")
    parser.add_argument("--max-frames", type=int, default=0, help="最多处理帧数，0为全部")
    parser.add_argument("--blue-cls", type=int, default=0, help="蓝色目标类别ID（默认0）")
    parser.add_argument("--red-cls", type=int, default=1, help="红色目标类别ID（默认1）")
    parser.add_argument(
        "--input-color",
        type=str,
        default="rgb",
        choices=["rgb", "bgr"],
        help="送入模型的通道顺序（默认rgb）",
    )
    return parser.parse_args()


def load_engine(engine_path: Path) -> tuple[trt.ICudaEngine, trt.IExecutionContext]:
    """@brief 加载 TensorRT engine，并自动跳过 Ultralytics 元数据头。"""
    raw = engine_path.read_bytes()
    if raw[:1] == b"/" or (len(raw) > 4 and raw[4:5] == b"{"):
        meta_len = struct.unpack_from("<I", raw, 0)[0]
        raw = raw[4 + meta_len :]
        print(f"[INFO] Skip Ultralytics metadata header: {meta_len} bytes")

    logger = trt.Logger(trt.Logger.WARNING)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(raw)
    if engine is None:
        raise RuntimeError("Failed to deserialize TensorRT engine")
    context = engine.create_execution_context()
    if context is None:
        raise RuntimeError("Failed to create TensorRT execution context")
    return engine, context


def main() -> None:
    args = parse_args()

    engine_path = Path(args.engine).expanduser().resolve()
    input_video = Path(args.input).expanduser().resolve()
    if not engine_path.exists():
        raise FileNotFoundError(f"Engine not found: {engine_path}")
    if not input_video.exists():
        raise FileNotFoundError(f"Input video not found: {input_video}")

    if args.output:
        output_video = Path(args.output).expanduser().resolve()
    else:
        output_video = input_video.with_name(input_video.stem + "_detected_trt.mp4")

    engine, context = load_engine(engine_path)

    in_shape = engine.get_binding_shape(0)   # (1,3,H,W)
    out_shape = engine.get_binding_shape(1)  # (1,300,6)
    in_h, in_w = int(in_shape[2]), int(in_shape[3])
    print(f"[INFO] Input shape: {tuple(in_shape)}, Output shape: {tuple(out_shape)}")

    inp = torch.zeros(list(in_shape), dtype=torch.float32, device="cuda")
    out = torch.zeros(list(out_shape), dtype=torch.float32, device="cuda")
    bindings = [inp.data_ptr(), out.data_ptr()]

    cap = cv2.VideoCapture(str(input_video))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {input_video}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    output_video.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output_video),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (src_w, src_h),
    )
    if not writer.isOpened():
        raise RuntimeError(f"Cannot open output writer: {output_video}")

    class_names = {args.blue_cls: "blue", args.red_cls: "red"}
    class_colors = {args.blue_cls: (255, 0, 0), args.red_cls: (0, 0, 255)}  # BGR

    print(f"[INFO] Input:  {input_video}")
    print(f"[INFO] Output: {output_video}")
    print(f"[INFO] FPS: {fps:.2f}, Size: {src_w}x{src_h}, Frames: {total_frames}")
    print("[INFO] Start infer...")

    t0 = time.perf_counter()
    frame_id = 0
    det_frames = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if args.max_frames > 0 and frame_id >= args.max_frames:
            break

        resized = cv2.resize(frame, (in_w, in_h))
        arr = resized.astype(np.float32) / 255.0
        if args.input_color == "rgb":
            arr = arr[:, :, ::-1].copy()  # BGR -> RGB
        arr = np.transpose(arr, (2, 0, 1))[None]

        inp.copy_(torch.from_numpy(arr).to("cuda"))
        context.execute_v2(bindings)
        pred = out.cpu().numpy()[0]  # (300,6): x1,y1,x2,y2,conf,cls

        sx = src_w / float(in_w)
        sy = src_h / float(in_h)
        has_det = False

        for det in pred:
            x1, y1, x2, y2, conf, cls = det
            if conf < args.conf:
                continue
            has_det = True

            cls_id = int(cls)
            if cls_id not in class_names:
                continue
            color = class_colors.get(cls_id, (0, 255, 255))
            name = class_names.get(cls_id, str(cls_id))

            x1 = int(max(0, min(src_w - 1, x1 * sx)))
            y1 = int(max(0, min(src_h - 1, y1 * sy)))
            x2 = int(max(0, min(src_w - 1, x2 * sx)))
            y2 = int(max(0, min(src_h - 1, y2 * sy)))
            if x2 <= x1 or y2 <= y1:
                continue

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            label = f"{name} {conf:.2f}"
            cv2.putText(
                frame,
                label,
                (x1, max(20, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                2,
            )

        if has_det:
            det_frames += 1

        writer.write(frame)
        frame_id += 1

        if frame_id % 100 == 0:
            print(f"[INFO] Processed {frame_id}/{total_frames if total_frames > 0 else '?'}")

    cap.release()
    writer.release()

    dt = time.perf_counter() - t0
    speed_fps = frame_id / dt if dt > 0 else 0.0
    print(f"[INFO] Done. Frames: {frame_id}, detection_frames: {det_frames}, speed: {speed_fps:.2f} FPS")
    print(f"[INFO] Saved: {output_video}")


if __name__ == "__main__":
    main()
