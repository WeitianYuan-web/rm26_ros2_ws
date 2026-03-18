#!/usr/bin/env bash
# ==============================================================================
# @file   check_env.sh
# @brief  Jetson Xavier NX 环境检查脚本
#
# 检查将 YOLO .pt 模型转换为 TensorRT .engine 所需的全部依赖，
# 并在缺少依赖时提供对应的安装指引。
#
# 用法:
#   chmod +x check_env.sh
#   ./check_env.sh
# ==============================================================================

set -euo pipefail

# ── 颜色定义 ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
BOLD='\033[1m'
NC='\033[0m'

OK="${GREEN}[  OK  ]${NC}"
MISS="${RED}[ MISS ]${NC}"
WARN="${YELLOW}[ WARN ]${NC}"
INFO="${BLUE}[ INFO ]${NC}"

MISSING_DEPS=()

# ── 辅助函数 ──────────────────────────────────────────────────────────────────
print_section() {
    echo ""
    echo -e "${BOLD}━━━ $1 ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
}

check_cmd() {
    local name="$1"
    local cmd="$2"
    if command -v "$cmd" &>/dev/null; then
        local ver
        ver=$(eval "$cmd --version 2>&1 | head -1" || echo "N/A")
        echo -e "${OK} ${name}: ${ver}"
        return 0
    else
        echo -e "${MISS} ${name} 未找到"
        MISSING_DEPS+=("$name")
        return 1
    fi
}

check_python_module() {
    local name="$1"
    local module="$2"
    local ver_cmd="${3:-import $2; print($2.__version__)}"
    if python3 -c "$ver_cmd" &>/dev/null 2>&1; then
        local ver
        ver=$(python3 -c "$ver_cmd" 2>&1)
        echo -e "${OK} Python/$name: ${ver}"
        return 0
    else
        echo -e "${MISS} Python/$name 未安装"
        MISSING_DEPS+=("python-$name")
        return 1
    fi
}

# ══════════════════════════════════════════════════════════════════════════════
print_section "Jetson 平台信息"
echo -e "${INFO} 架构: $(uname -m)"
if [[ -f /etc/nv_tegra_release ]]; then
    echo -e "${OK} JetPack/L4T: $(cat /etc/nv_tegra_release | head -1)"
else
    echo -e "${WARN} 未找到 /etc/nv_tegra_release，可能不是 Jetson 平台"
fi
if [[ -f /etc/os-release ]]; then
    . /etc/os-release
    echo -e "${INFO} OS: ${PRETTY_NAME}"
fi

# ══════════════════════════════════════════════════════════════════════════════
print_section "CUDA 工具链"
if command -v nvcc &>/dev/null; then
    CUDA_VER=$(nvcc --version 2>&1 | grep -oP "release \K[0-9.]+")
    echo -e "${OK} CUDA (nvcc): ${CUDA_VER}"
else
    echo -e "${MISS} nvcc 未找到（CUDA 未安装或未加入 PATH）"
    MISSING_DEPS+=("cuda-toolkit")
fi

# 检测 CUDA 运行时库
if ldconfig -p 2>/dev/null | grep -q "libcuda.so"; then
    echo -e "${OK} libcuda.so 已找到"
else
    echo -e "${WARN} ldconfig 中未找到 libcuda.so"
fi

# ══════════════════════════════════════════════════════════════════════════════
print_section "cuDNN"
CUDNN_H=$(find /usr/include -name "cudnn_version.h" 2>/dev/null | head -1)
if [[ -n "$CUDNN_H" ]]; then
    CUDNN_MAJOR=$(grep "CUDNN_MAJOR" "$CUDNN_H" 2>/dev/null | head -1 | awk '{print $3}')
    CUDNN_MINOR=$(grep "CUDNN_MINOR" "$CUDNN_H" 2>/dev/null | head -1 | awk '{print $3}')
    CUDNN_PATCH=$(grep "CUDNN_PATCHLEVEL" "$CUDNN_H" 2>/dev/null | head -1 | awk '{print $3}')
    echo -e "${OK} cuDNN: ${CUDNN_MAJOR}.${CUDNN_MINOR}.${CUDNN_PATCH}"
else
    DNN_DEB=$(dpkg-query -W -f='${Version}' libcudnn8 2>/dev/null)
    if [[ -n "$DNN_DEB" ]]; then
        echo -e "${OK} cuDNN (deb): ${DNN_DEB}"
    else
        echo -e "${MISS} cuDNN 未安装"
        MISSING_DEPS+=("libcudnn8")
    fi
fi

# ══════════════════════════════════════════════════════════════════════════════
print_section "TensorRT"
TRT_FOUND=false

# 方式1: dpkg
TRT_DEB=$(dpkg-query -W -f='${Version}' libnvinfer-dev 2>/dev/null || true)
if [[ -n "$TRT_DEB" ]]; then
    echo -e "${OK} TensorRT (libnvinfer-dev deb): ${TRT_DEB}"
    TRT_FOUND=true
fi

# 方式2: 库文件
TRT_LIB=$(find /usr/lib/aarch64-linux-gnu /usr/lib/x86_64-linux-gnu /usr/local/lib \
           -name "libnvinfer.so*" 2>/dev/null | head -1 || true)
if [[ -n "$TRT_LIB" ]]; then
    echo -e "${OK} libnvinfer.so: ${TRT_LIB}"
    TRT_FOUND=true
fi

# 方式3: trtexec
if command -v trtexec &>/dev/null; then
    echo -e "${OK} trtexec: $(trtexec --version 2>&1 | head -1)"
    TRT_FOUND=true
fi

# 方式4: Python 绑定
if python3 -c "import tensorrt as trt; print(trt.__version__)" &>/dev/null 2>&1; then
    TRT_PY=$(python3 -c "import tensorrt as trt; print(trt.__version__)")
    echo -e "${OK} Python/tensorrt: ${TRT_PY}"
    TRT_FOUND=true
else
    echo -e "${MISS} Python/tensorrt 未安装"
    MISSING_DEPS+=("python-tensorrt")
fi

if [[ "$TRT_FOUND" == "false" ]]; then
    echo -e "${MISS} TensorRT 完全未安装"
    MISSING_DEPS+=("tensorrt")
fi

# ══════════════════════════════════════════════════════════════════════════════
print_section "Python 环境"
PYTHON_BIN=$(which python3)
PYTHON_VER=$(python3 --version 2>&1)
echo -e "${INFO} Python 解释器: ${PYTHON_BIN}"
echo -e "${INFO} Python 版本: ${PYTHON_VER}"

check_python_module "numpy" "numpy"
check_python_module "torch" "torch" \
    "import torch; print(torch.__version__ + ' | CUDA=' + str(torch.cuda.is_available()))"
check_python_module "onnx" "onnx"
check_python_module "ultralytics" "ultralytics"
check_python_module "cv2 (OpenCV)" "cv2"

# PyCUDA (可选)
if python3 -c "import pycuda; print(pycuda.VERSION_TEXT)" &>/dev/null 2>&1; then
    PY_CUDA=$(python3 -c "import pycuda; print(pycuda.VERSION_TEXT)")
    echo -e "${OK} Python/pycuda: ${PY_CUDA}"
else
    echo -e "${WARN} Python/pycuda 未安装 (可选，用于自定义 TRT 推理)"
fi

# ══════════════════════════════════════════════════════════════════════════════
print_section "模型文件检查"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODEL_PATH="${SCRIPT_DIR}/../model/yolo26n_rm_500.pt"
if [[ -f "$MODEL_PATH" ]]; then
    SIZE=$(du -sh "$MODEL_PATH" | cut -f1)
    echo -e "${OK} 模型文件: $(realpath "$MODEL_PATH") (${SIZE})"
else
    echo -e "${MISS} 模型文件不存在: ${MODEL_PATH}"
    MISSING_DEPS+=("model-file")
fi

# ══════════════════════════════════════════════════════════════════════════════
print_section "检查结果汇总"
if [[ ${#MISSING_DEPS[@]} -eq 0 ]]; then
    echo -e "${GREEN}${BOLD}✓ 所有依赖已满足，可以运行 pt2trt.py 进行转换${NC}"
else
    echo -e "${RED}${BOLD}✗ 以下依赖缺失: ${MISSING_DEPS[*]}${NC}"
    echo ""
    echo -e "${BOLD}── 安装指引 (Jetson Xavier NX / JetPack 5.x / CUDA 11.4) ────────────────${NC}"

    for dep in "${MISSING_DEPS[@]}"; do
        case "$dep" in
        tensorrt | python-tensorrt)
            cat <<'EOF'
  [TensorRT]
  # 方式一: apt 安装 (推荐，JetPack 已集成时可用)
  sudo apt-get update
  sudo apt-get install -y tensorrt python3-libnvinfer python3-libnvinfer-dev

  # 方式二: pip 安装 Python 绑定 (需要先安装 libnvinfer deb 包)
  pip3 install tensorrt
EOF
            ;;
        python-torch)
            cat <<'EOF'
  [PyTorch for Jetson Xavier NX - JetPack 5.x, CUDA 11.4]
  # 从 NVIDIA 官方页面下载对应轮子:
  # https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/
  # 推荐: torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
  wget https://developer.download.nvidia.com/compute/redist/jp/v512/pytorch/torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
  pip3 install torch-2.1.0a0+41361538.nv23.06-cp38-cp38-linux_aarch64.whl
  # 对应 torchvision:
  # https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
EOF
            ;;
        python-onnx)
            echo "  [ONNX]"
            echo "  pip3 install onnx onnxruntime"
            ;;
        python-ultralytics)
            echo "  [Ultralytics YOLOv8]"
            echo "  pip3 install ultralytics"
            ;;
        libcudnn8)
            echo "  [cuDNN]"
            echo "  # JetPack 5.x 应已预装 cuDNN 8.6，请重新刷写或检查 JetPack 版本"
            ;;
        esac
    done

    echo ""
    echo -e "${WARN} 安装完成后请重新运行本脚本确认环境就绪"
fi
echo ""
