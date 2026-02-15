#!/usr/bin/env python3
"""
棋盘格标定板生成脚本 — 生成适合 A4 纸打印的高精度棋盘格图像

使用方法:
    python3 generate_chessboard.py                    # 默认 9x6 内角点, 25mm 方格
    python3 generate_chessboard.py --rows 7 --cols 10 --square 30
    python3 generate_chessboard.py --output my_board.png --dpi 600

输出文件会在当前目录生成 PNG 图像，按实际尺寸 1:1 打印即可。

打印注意事项:
    1. 打印时选择「实际大小」/「100%」缩放，不要选「适合页面」
    2. 打印后用尺子量几个格子，确认尺寸与 --square 参数一致
    3. 建议贴在硬纸板或亚克力板上保持平整
"""

import argparse
import numpy as np

try:
    import cv2
except ImportError:
    print("错误: 需要 OpenCV, 请运行 pip3 install opencv-python")
    exit(1)


def generate_chessboard(rows: int, cols: int, square_mm: float,
                         dpi: int, margin_mm: float) -> np.ndarray:
    """
    生成棋盘格图像

    Args:
        rows: 内角点行数 (棋盘格行数 = rows + 1)
        cols: 内角点列数 (棋盘格列数 = cols + 1)
        square_mm: 每个方格边长 (mm)
        dpi: 输出图像 DPI
        margin_mm: 页面边距 (mm)

    Returns:
        棋盘格图像 (numpy array)
    """
    # 像素 / mm 转换系数
    px_per_mm = dpi / 25.4

    # 棋盘格区域尺寸
    board_rows = rows + 1  # 方格行数
    board_cols = cols + 1  # 方格列数
    board_w_mm = board_cols * square_mm
    board_h_mm = board_rows * square_mm

    # A4 纸尺寸 (mm): 210 x 297
    a4_w_mm = 210.0
    a4_h_mm = 297.0

    # 检查是否需要横向放置
    landscape = False
    if (board_w_mm + 2 * margin_mm > a4_w_mm or
            board_h_mm + 2 * margin_mm > a4_h_mm):
        # 尝试横向
        if (board_w_mm + 2 * margin_mm <= a4_h_mm and
                board_h_mm + 2 * margin_mm <= a4_w_mm):
            landscape = True
            a4_w_mm, a4_h_mm = a4_h_mm, a4_w_mm
            print(f"  棋盘格较大，自动切换为横向 (Landscape) 布局")
        else:
            print(f"  警告: 棋盘格 ({board_w_mm:.0f}x{board_h_mm:.0f}mm) "
                  f"超出 A4 纸可打印区域!")

    # 总图像尺寸 (像素)
    img_w = int(round(a4_w_mm * px_per_mm))
    img_h = int(round(a4_h_mm * px_per_mm))

    # 创建白色底图
    img = np.ones((img_h, img_w), dtype=np.uint8) * 255

    # 棋盘格起始位置 (居中)
    offset_x = int(round((a4_w_mm - board_w_mm) / 2.0 * px_per_mm))
    offset_y = int(round((a4_h_mm - board_h_mm) / 2.0 * px_per_mm))

    square_px = int(round(square_mm * px_per_mm))

    # 绘制棋盘格
    for r in range(board_rows):
        for c in range(board_cols):
            if (r + c) % 2 == 0:
                continue  # 白色格子，跳过
            x1 = offset_x + c * square_px
            y1 = offset_y + r * square_px
            x2 = x1 + square_px
            y2 = y1 + square_px
            img[y1:y2, x1:x2] = 0  # 黑色

    # 在底部添加标注文字
    info_text = (f"Chessboard {cols}x{rows} inner corners | "
                 f"Square: {square_mm:.1f}mm | "
                 f"Board: {board_w_mm:.0f}x{board_h_mm:.0f}mm")
    font_scale = 0.4 * (dpi / 300.0)
    thickness = max(1, int(dpi / 300.0))
    text_size = cv2.getTextSize(info_text, cv2.FONT_HERSHEY_SIMPLEX,
                                font_scale, thickness)[0]
    text_x = (img_w - text_size[0]) // 2
    text_y = img_h - int(margin_mm * 0.3 * px_per_mm)
    cv2.putText(img, info_text, (text_x, text_y),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, 0, thickness)

    return img, landscape


def main():
    parser = argparse.ArgumentParser(
        description="生成 A4 纸打印用棋盘格标定板",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("--rows", type=int, default=6,
                        help="内角点行数 (默认: 6)")
    parser.add_argument("--cols", type=int, default=9,
                        help="内角点列数 (默认: 9)")
    parser.add_argument("--square", type=float, default=25.0,
                        help="方格边长 mm (默认: 25.0)")
    parser.add_argument("--dpi", type=int, default=300,
                        help="输出 DPI (默认: 300)")
    parser.add_argument("--margin", type=float, default=10.0,
                        help="页面最小边距 mm (默认: 10.0)")
    parser.add_argument("--output", type=str, default=None,
                        help="输出文件名 (默认: chessboard_<cols>x<rows>.png)")
    args = parser.parse_args()

    if args.output is None:
        args.output = f"chessboard_{args.cols}x{args.rows}.png"

    print(f"正在生成棋盘格标定板...")
    print(f"  内角点: {args.cols} x {args.rows}")
    print(f"  方格数: {args.cols + 1} x {args.rows + 1}")
    print(f"  方格边长: {args.square} mm")
    print(f"  DPI: {args.dpi}")

    img, landscape = generate_chessboard(
        rows=args.rows,
        cols=args.cols,
        square_mm=args.square,
        dpi=args.dpi,
        margin_mm=args.margin,
    )

    cv2.imwrite(args.output, img)

    board_w = (args.cols + 1) * args.square
    board_h = (args.rows + 1) * args.square
    orient = "横向 (Landscape)" if landscape else "纵向 (Portrait)"

    print(f"\n已生成: {args.output}")
    print(f"  图像尺寸: {img.shape[1]} x {img.shape[0]} 像素")
    print(f"  棋盘格区域: {board_w:.0f} x {board_h:.0f} mm")
    print(f"  打印方向: {orient}")
    print(f"\n打印提示:")
    print(f"  1. 选择「实际大小」/「100%」打印，不要缩放")
    print(f"  2. 打印后量一下格子，确认边长 = {args.square} mm")
    print(f"  3. 贴在硬纸板上保持平整")
    print(f"\n标定时使用:")
    print(f"  python3 camera_calibration.py "
          f"--rows {args.rows} --cols {args.cols} --square {args.square}")


if __name__ == "__main__":
    main()
