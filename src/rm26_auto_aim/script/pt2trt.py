#!/usr/bin/env python3
# ==============================================================================
# @file   pt2trt.py
# @brief  YOLO .pt 模型转 TensorRT .engine 转换脚本
#
# 支持三种转换路径:
#   1. direct     — 通过 Ultralytics API 直接导出 TensorRT engine (需要 CUDA)
#   2. onnx       — .pt → ONNX → TensorRT engine (兼容性更强)
#   3. from-onnx  — 直接使用已有 ONNX 文件转 engine (跳过 .pt 步骤)
#
# 适用平台: Jetson Xavier NX / JetPack 5.x / CUDA 11.4 / TensorRT 8.5.x
#
# 用法示例:
#   # 环境检查（不做转换）
#   python3 pt2trt.py --check-only
#
#   # 默认转换（FP16，640×640，自动选择路径）
#   python3 pt2trt.py
#
#   # 直接使用已有 ONNX 转 engine（跳过 .pt 导出步骤）
#   python3 pt2trt.py --from-onnx ../model/yolo26n_rm_500.onnx
#
#   # 指定参数
#   python3 pt2trt.py --model ../model/yolo26n_rm_500.pt \
#                     --output ../model/yolo26n_rm_500.engine \
#                     --imgsz 640 --fp16 --method onnx
#
#   # 仅导出 ONNX（不继续转 TRT）
#   python3 pt2trt.py --method onnx --onnx-only
# ==============================================================================

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import os
from pathlib import Path


# ──────────────────────────────────────────────────────────────────────────────
# 颜色输出
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
    if title:
        print(f"\n{C.HEAD}── {title} {'─' * max(0, 50 - len(title))}{C.END}")
    else:
        print("─" * 56)


# ──────────────────────────────────────────────────────────────────────────────
# 环境检查
# ──────────────────────────────────────────────────────────────────────────────
def check_dependencies() -> list[str]:
    """
    @brief 逐项检查转换所需依赖，返回缺失依赖列表。
    @return 缺失依赖名称列表，列表为空则表示环境完整。
    """
    missing: list[str] = []
    hline("依赖检查")

    # Python 版本
    pyver = sys.version.split()[0]
    print(f"{C.INFO} Python: {pyver}")
    if tuple(int(x) for x in pyver.split(".")[:2]) < (3, 8):
        print(f"{C.WARN} 建议使用 Python >= 3.8")

    # PyTorch
    try:
        import torch
        cuda_ok = torch.cuda.is_available()
        gpu_name = torch.cuda.get_device_name(0) if cuda_ok else "N/A"
        is_jetson_torch = "+nv" in torch.__version__ or "nv23" in torch.__version__
        print(f"{C.OK} PyTorch: {torch.__version__}  |  CUDA={cuda_ok}  |  GPU={gpu_name}")
        if not cuda_ok:
            if not is_jetson_torch:
                print(f"{C.WARN} 当前 PyTorch 为 CPU-only 版本（由 pip/PyPI 安装）")
                print(f"{C.WARN} 请卸载后安装 NVIDIA Jetson 专用版本：")
                print(f"         pip3 uninstall torch -y")
                print(f"         pip3 install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl --no-deps")
            else:
                print(f"{C.WARN} CUDA 不可用，请检查 /dev/nvidia* 设备权限和驱动状态")
    except ImportError:
        print(f"{C.MISS} PyTorch 未安装")
        missing.append("torch")

    # TensorRT Python 绑定
    try:
        import tensorrt as trt
        print(f"{C.OK} TensorRT: {trt.__version__}")
    except ImportError:
        print(f"{C.MISS} TensorRT Python 绑定未安装")
        missing.append("tensorrt")

    # ONNX
    try:
        import onnx
        print(f"{C.OK} ONNX: {onnx.__version__}")
    except ImportError:
        print(f"{C.MISS} ONNX 未安装")
        missing.append("onnx")

    # Ultralytics
    try:
        import ultralytics
        print(f"{C.OK} Ultralytics: {ultralytics.__version__}")
    except ImportError:
        print(f"{C.MISS} Ultralytics 未安装")
        missing.append("ultralytics")

    # OpenCV（可选）
    try:
        import cv2
        print(f"{C.OK} OpenCV: {cv2.__version__}")
    except ImportError:
        print(f"{C.WARN} OpenCV 未安装（可选）")

    return missing


def print_install_guide() -> None:
    """
    @brief 输出针对 Jetson Xavier NX / JetPack 5.x 的依赖安装指引。
    """
    hline("安装指引  (Jetson Xavier NX · JetPack 5.x · CUDA 11.4)")
    print("""
  ① TensorRT（apt 方式，推荐）:
     sudo apt-get update
     sudo apt-get install -y tensorrt python3-libnvinfer python3-libnvinfer-dev \\
         libnvinfer-dev libnvonnxparsers-dev libnvparsers-dev

  ② PyTorch for Jetson Xavier NX (JetPack 5.x / Python 3.8):
     # 下载 NVIDIA 官方 aarch64 whl，访问：
     # https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/
     wget https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/\\
torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
     pip3 install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl

  ③ ONNX:
     pip3 install onnx onnxruntime

  ④ Ultralytics:
     pip3 install ultralytics

  ⑤ （可选）onnx-simplifier:
     pip3 install onnxsim

  安装完成后运行:  python3 pt2trt.py --check-only
""")


# ──────────────────────────────────────────────────────────────────────────────
# 转换：direct（Ultralytics 直接导出 engine）
# ──────────────────────────────────────────────────────────────────────────────
def convert_direct(
    model_path: str,
    output_path: str,
    imgsz: int,
    fp16: bool,
    workspace_gb: int,
) -> None:
    """
    @brief 通过 Ultralytics YOLO.export() 直接生成 TensorRT engine。
    @param model_path    输入 .pt 文件路径
    @param output_path   输出 .engine 文件路径
    @param imgsz         推理图像尺寸（正方形边长）
    @param fp16          是否使用 FP16 精度
    @param workspace_gb  TensorRT 构建时最大工作空间 (GB)
    """
    from ultralytics import YOLO  # type: ignore

    hline("Ultralytics 直接导出 TensorRT engine")
    print(f"{C.INFO} 加载模型: {model_path}")
    model = YOLO(model_path)

    print(f"{C.INFO} 导出参数: imgsz={imgsz}, fp16={fp16}, workspace={workspace_gb}GB, device=0")
    model.export(
        format="engine",
        imgsz=imgsz,
        half=fp16,
        device=0,
        workspace=workspace_gb,
        simplify=True,
    )

    # Ultralytics 默认将 .engine 存放在与 .pt 同级目录
    default_out = Path(model_path).with_suffix(".engine")
    if default_out.exists() and str(default_out.resolve()) != str(Path(output_path).resolve()):
        Path(output_path).parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(default_out), output_path)

    if not Path(output_path).exists():
        raise FileNotFoundError(f"未找到导出结果: {output_path}")

    size_mb = Path(output_path).stat().st_size / 1024 / 1024
    print(f"{C.OK} engine 已保存: {output_path}  ({size_mb:.1f} MB)")


# ──────────────────────────────────────────────────────────────────────────────
# 转换：ONNX 中间格式路径
# ──────────────────────────────────────────────────────────────────────────────
def export_to_onnx(
    model_path: str,
    output_dir: str,
    imgsz: int,
) -> str:
    """
    @brief 将 .pt 模型导出为 ONNX 格式。
    @param model_path  输入 .pt 文件路径
    @param output_dir  ONNX 文件输出目录
    @param imgsz       推理图像尺寸
    @return 导出的 ONNX 文件绝对路径
    """
    from ultralytics import YOLO  # type: ignore

    hline("Step 1/2  导出 ONNX")
    onnx_name = Path(model_path).stem + ".onnx"
    onnx_path = str(Path(output_dir) / onnx_name)

    print(f"{C.INFO} 加载模型: {model_path}")
    model = YOLO(model_path)

    print(f"{C.INFO} 导出 ONNX: imgsz={imgsz}, opset=11, simplify=True")
    model.export(
        format="onnx",
        imgsz=imgsz,
        opset=11,
        simplify=True,
        dynamic=False,
        half=False,
    )

    # 移动到目标目录
    default_onnx = Path(model_path).with_suffix(".onnx")
    if default_onnx.exists():
        if str(default_onnx.resolve()) != str(Path(onnx_path).resolve()):
            Path(output_dir).mkdir(parents=True, exist_ok=True)
            shutil.move(str(default_onnx), onnx_path)
    elif not Path(onnx_path).exists():
        raise FileNotFoundError(f"ONNX 导出后未找到文件: {onnx_path}")

    size_mb = Path(onnx_path).stat().st_size / 1024 / 1024
    print(f"{C.OK} ONNX 已保存: {onnx_path}  ({size_mb:.1f} MB)")
    return onnx_path


def onnx_to_trt(
    onnx_path: str,
    output_path: str,
    fp16: bool,
    workspace_gb: int,
) -> None:
    """
    @brief 通过 TensorRT Python API 将 ONNX 转换为序列化 engine 文件。
    @param onnx_path    输入 ONNX 文件路径
    @param output_path  输出 .engine 文件路径
    @param fp16         是否启用 FP16 精度
    @param workspace_gb TensorRT 构建工作空间大小 (GB)
    """
    import tensorrt as trt  # type: ignore

    hline("Step 2/2  TensorRT 构建 engine")
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)

    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network(
        1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    )
    parser = trt.OnnxParser(network, TRT_LOGGER)

    print(f"{C.INFO} 解析 ONNX: {onnx_path}")
    with open(onnx_path, "rb") as f:
        raw = f.read()
    if not parser.parse(raw):
        errors = [str(parser.get_error(i)) for i in range(parser.num_errors)]
        raise RuntimeError("ONNX 解析失败:\n" + "\n".join(errors))
    print(f"{C.OK} ONNX 解析成功，网络层数: {network.num_layers}")

    config = builder.create_builder_config()
    workspace_bytes = workspace_gb * (1 << 30)

    # TensorRT 8.x / 9.x API 兼容
    if hasattr(trt, "MemoryPoolType"):
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, workspace_bytes)
    else:
        config.max_workspace_size = workspace_bytes  # type: ignore[attr-defined]

    if fp16:
        if builder.platform_has_fast_fp16:
            config.set_flag(trt.BuilderFlag.FP16)
            print(f"{C.OK} FP16 精度已启用")
        else:
            print(f"{C.WARN} 当前平台不支持 FP16，回退到 FP32")

    print(f"{C.INFO} 正在构建 TensorRT engine，请耐心等待（通常 3-10 分钟）...")
    try:
        serialized = builder.build_serialized_network(network, config)
    except Exception as e:
        raise RuntimeError(
            f"TensorRT Python API 构建失败: {e}\n"
            "  提示: 可尝试 trtexec 命令行方式，脚本将自动回退"
        )
    if serialized is None:
        raise RuntimeError("TensorRT engine 构建失败，请检查 ONNX 模型与 TensorRT 版本兼容性")

    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "wb") as f:
        f.write(serialized)

    size_mb = Path(output_path).stat().st_size / 1024 / 1024
    print(f"{C.OK} engine 已保存: {output_path}  ({size_mb:.1f} MB)")


# ──────────────────────────────────────────────────────────────────────────────
# trtexec 备用路径（当 Python TRT API 不可用时）
# ──────────────────────────────────────────────────────────────────────────────
def find_trtexec() -> str | None:
    """
    @brief 在系统中搜索 trtexec 可执行文件。
    @return trtexec 路径，未找到则返回 None。
    """
    found = shutil.which("trtexec")
    if found:
        return found
    candidates = [
        "/usr/src/tensorrt/bin/trtexec",
        "/usr/local/tensorrt/bin/trtexec",
        "/opt/tensorrt/bin/trtexec",
        "/usr/bin/trtexec",
    ]
    for c in candidates:
        if Path(c).exists():
            return c
    return None


def trtexec_convert(
    onnx_path: str,
    output_path: str,
    fp16: bool,
    workspace_gb: int,
) -> None:
    """
    @brief 调用 trtexec 命令行工具将 ONNX 转换为 engine。
    @param onnx_path    输入 ONNX 文件路径
    @param output_path  输出 .engine 文件路径
    @param fp16         是否启用 FP16 精度
    @param workspace_gb 工作空间大小 (GB)
    """
    trtexec_bin = find_trtexec()

    if not trtexec_bin:
        raise FileNotFoundError(
            "未找到 trtexec，请安装 TensorRT 或手动指定路径\n"
            "  Jetson 默认路径: /usr/src/tensorrt/bin/trtexec"
        )

    hline("使用 trtexec 构建 engine")
    workspace_mb = workspace_gb * 1024
    cmd = [
        trtexec_bin,
        f"--onnx={onnx_path}",
        f"--saveEngine={output_path}",
        f"--workspace={workspace_mb}",
    ]
    if fp16:
        cmd.append("--fp16")

    print(f"{C.INFO} 执行命令: {' '.join(cmd)}")
    result = subprocess.run(cmd, capture_output=False)
    if result.returncode != 0:
        raise RuntimeError(f"trtexec 返回非零退出码: {result.returncode}")
    print(f"{C.OK} engine 构建完成: {output_path}")


# ──────────────────────────────────────────────────────────────────────────────
# 内部辅助：统一的 ONNX → engine 入口（自动回退 Python API → trtexec）
# ──────────────────────────────────────────────────────────────────────────────
def _run_onnx_to_trt(
    onnx_path: str,
    output_path: str,
    fp16: bool,
    workspace_gb: int,
) -> None:
    """
    @brief 尝试 Python TRT API，失败则自动回退到 trtexec 命令行工具。
    @param onnx_path    输入 ONNX 文件路径
    @param output_path  输出 .engine 文件路径
    @param fp16         是否启用 FP16
    @param workspace_gb 工作空间大小 (GB)
    """
    try:
        import tensorrt  # noqa: F401
        onnx_to_trt(onnx_path, output_path, fp16, workspace_gb)
    except (ImportError, RuntimeError, Exception) as e:
        err_msg = str(e)
        trtexec_bin = find_trtexec()
        if trtexec_bin:
            print(f"{C.WARN} Python TRT API 不可用: {err_msg}")
            print(f"{C.INFO} 自动回退到 trtexec: {trtexec_bin}")
            trtexec_convert(onnx_path, output_path, fp16, workspace_gb)
        else:
            raise RuntimeError(
                f"Python TRT API 失败且 trtexec 未找到。\n"
                f"原始错误: {err_msg}\n"
                f"请确认 TensorRT 已正确安装，或手动运行:\n"
                f"  /usr/src/tensorrt/bin/trtexec --onnx={onnx_path} "
                f"--saveEngine={output_path} {'--fp16' if fp16 else ''}"
            )


def _print_done(output_path: str) -> None:
    """@brief 打印转换完成信息。"""
    hline("转换完成")
    if Path(output_path).exists():
        size_mb = Path(output_path).stat().st_size / 1024 / 1024
        print(f"{C.OK} 输出文件: {output_path}  ({size_mb:.1f} MB)")
        print(f"\n{C.INFO} 使用示例（Python 推理验证）:")
        print(f"   from ultralytics import YOLO")
        print(f"   model = YOLO('{output_path}')")
        print(f"   results = model.predict('image.jpg', device=0)")
    print()


# ──────────────────────────────────────────────────────────────────────────────
# 主入口
# ──────────────────────────────────────────────────────────────────────────────
def parse_args() -> argparse.Namespace:
    """@brief 解析命令行参数。"""
    script_dir = Path(__file__).resolve().parent
    default_model = str(script_dir / "../model/yolo26n_rm_500_n.pt")

    parser = argparse.ArgumentParser(
        description="YOLO .pt → TensorRT .engine  (Jetson Xavier NX)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--model", "-m",
        type=str,
        default=default_model,
        help="输入 .pt 模型路径\n(默认: ../model/yolo26n_rm_500.pt)",
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default=None,
        help="输出 .engine 路径\n(默认: 与模型同目录，同名 .engine)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="推理图像边长，像素 (默认: 640)",
    )
    parser.add_argument(
        "--fp16",
        action="store_true",
        default=True,
        help="启用 FP16 精度（Jetson 推荐，默认开启）",
    )
    parser.add_argument(
        "--no-fp16",
        dest="fp16",
        action="store_false",
        help="禁用 FP16，使用 FP32",
    )
    parser.add_argument(
        "--workspace",
        type=int,
        default=4,
        metavar="GB",
        help="TensorRT 构建工作空间大小 GB (默认: 4)",
    )
    parser.add_argument(
        "--method",
        type=str,
        default="auto",
        choices=["auto", "direct", "onnx"],
        help=(
            "转换方式:\n"
            "  auto   - 优先 direct，失败后自动回退到 onnx (默认)\n"
            "  direct - Ultralytics 直接导出 engine\n"
            "  onnx   - .pt → ONNX → TensorRT engine"
        ),
    )
    parser.add_argument(
        "--from-onnx",
        type=str,
        default=None,
        metavar="ONNX_PATH",
        help=(
            "直接使用指定的 ONNX 文件转换为 engine，跳过 .pt 导出步骤\n"
            "示例: --from-onnx ../model/yolo26n_rm_500.onnx"
        ),
    )
    parser.add_argument(
        "--onnx-only",
        action="store_true",
        default=False,
        help="仅导出 ONNX，不转换为 engine（--method onnx 时有效）",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        default=False,
        help="仅检查环境依赖，不执行转换",
    )
    return parser.parse_args()


def main() -> None:
    print(f"\n{C.HEAD}{'═' * 56}")
    print("  YOLO .pt  →  TensorRT .engine  转换工具")
    print(f"  Jetson Xavier NX  |  CUDA 11.4  |  TensorRT 8.5.x")
    print(f"{'═' * 56}{C.END}")

    args = parse_args()

    # ── 环境检查 ──────────────────────────────────────────────────────────────
    missing = check_dependencies()

    if args.check_only:
        if missing:
            print_install_guide()
        else:
            print(f"\n{C.OK} 所有依赖满足，可运行转换。")
        sys.exit(0 if not missing else 1)

    # --from-onnx 模式：不需要 torch/ultralytics，仅需 TensorRT
    if args.from_onnx:
        onnx_input = str(Path(args.from_onnx).resolve())
        if not Path(onnx_input).exists():
            print(f"{C.ERR} ONNX 文件不存在: {onnx_input}")
            sys.exit(1)

        output_path = (
            str(Path(onnx_input).with_suffix(".engine"))
            if args.output is None
            else str(Path(args.output).resolve())
        )

        hline("转换配置  (--from-onnx 模式)")
        print(f"  输入 ONNX : {onnx_input}")
        print(f"  输出路径  : {output_path}")
        print(f"  精度模式  : {'FP16' if args.fp16 else 'FP32'}")
        print(f"  工作空间  : {args.workspace} GB")

        _run_onnx_to_trt(onnx_input, output_path, args.fp16, args.workspace)
        _print_done(output_path)
        return

    if missing:
        print(f"\n{C.ERR} 缺少依赖: {missing}")
        print_install_guide()
        sys.exit(1)

    # ── 路径解析 ──────────────────────────────────────────────────────────────
    model_path = str(Path(args.model).resolve())
    if not Path(model_path).exists():
        print(f"{C.ERR} 模型文件不存在: {model_path}")
        sys.exit(1)

    output_path = (
        str(Path(model_path).with_suffix(".engine"))
        if args.output is None
        else str(Path(args.output).resolve())
    )

    hline("转换配置")
    print(f"  输入模型  : {model_path}")
    print(f"  输出路径  : {output_path}")
    print(f"  图像尺寸  : {args.imgsz} × {args.imgsz}")
    print(f"  精度模式  : {'FP16' if args.fp16 else 'FP32'}")
    print(f"  工作空间  : {args.workspace} GB")
    print(f"  转换方式  : {args.method}")

    # ── 执行转换 ──────────────────────────────────────────────────────────────
    try:
        if args.method in ("direct", "auto"):
            try:
                convert_direct(model_path, output_path, args.imgsz, args.fp16, args.workspace)
            except Exception as exc:
                if args.method == "auto":
                    print(f"{C.WARN} direct 方式失败: {exc}")
                    print(f"{C.INFO} 回退到 ONNX 中间格式路径...")
                    out_dir = str(Path(output_path).parent)
                    onnx_path = export_to_onnx(model_path, out_dir, args.imgsz)
                    if not args.onnx_only:
                        _run_onnx_to_trt(onnx_path, output_path, args.fp16, args.workspace)
                else:
                    raise

        elif args.method == "onnx":
            out_dir = str(Path(output_path).parent)
            onnx_path = export_to_onnx(model_path, out_dir, args.imgsz)
            if not args.onnx_only:
                _run_onnx_to_trt(onnx_path, output_path, args.fp16, args.workspace)

    except Exception as exc:
        print(f"\n{C.ERR} 转换失败: {exc}")
        sys.exit(1)

    # ── 完成 ──────────────────────────────────────────────────────────────────
    _print_done(output_path)


if __name__ == "__main__":
    main()
