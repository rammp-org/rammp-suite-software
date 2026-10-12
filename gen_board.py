#!/usr/bin/env python3
"""Generate a print-ready ChArUco board PNG. Defaults: the RAMMP A2 board.

  uv run gen_board.py --out board_a2.png

Print at 100% scale ("actual size", never "fit to page") on MATTE paper,
mount on a rigid flat substrate (Dibond / 6mm+ MDF), then measure a run of
~10 squares with calipers and divide — the measured square size is what
feeds solve_calibration.py --square-size, not the nominal one.
"""

import argparse
from pathlib import Path

import cv2

from solve_calibration import charuco_board


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--cols", type=int, default=10, help="squares per row")
    ap.add_argument("--rows", type=int, default=14, help="squares per column")
    ap.add_argument("--square-mm", type=float, default=40.0)
    ap.add_argument("--marker-mm", type=float, default=30.0)
    ap.add_argument("--dict", default="4X4_100")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--margin-mm", type=float, default=10.0)
    ap.add_argument("--out", type=Path, default=Path("board.png"))
    args = ap.parse_args()

    board = charuco_board(
        args.cols, args.rows, args.square_mm / 1e3, args.marker_mm / 1e3, args.dict
    )
    px_per_mm = args.dpi / 25.4
    margin_px = round(args.margin_mm * px_per_mm)
    w = round(args.cols * args.square_mm * px_per_mm) + 2 * margin_px
    h = round(args.rows * args.square_mm * px_per_mm) + 2 * margin_px
    img = board.generateImage((w, h), marginSize=margin_px, borderBits=1)
    cv2.imwrite(str(args.out), img)

    width_mm = args.cols * args.square_mm + 2 * args.margin_mm
    height_mm = args.rows * args.square_mm + 2 * args.margin_mm
    print(f"wrote {args.out}: {w}x{h} px at {args.dpi} dpi")
    print(f"physical size {width_mm:.0f} x {height_mm:.0f} mm "
          f"({args.cols}x{args.rows} squares of {args.square_mm} mm, "
          f"{args.marker_mm} mm markers, DICT_{args.dict})")
    print("print at 100% scale, matte; mount flat; caliper-measure the squares.")
    print(
        "solve with: --board charuco "
        f"--cols {args.cols} --rows {args.rows} "
        f"--square-size <measured, ~{args.square_mm / 1e3:.3f}> "
        f"--marker-size <measured, ~{args.marker_mm / 1e3:.3f}> --dict {args.dict}"
    )


if __name__ == "__main__":
    main()
