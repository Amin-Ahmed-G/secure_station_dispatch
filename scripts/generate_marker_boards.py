#!/usr/bin/env python3
"""
generate_marker_boards.py
─────────────────────────
Generates every signage texture used by ``worlds/industrial_station_world.sdf``.

Each station bay carries a three-part sign stack, mirroring how fiducials are
actually deployed in a working facility:

    ┌────────────────────────────────────────┐  nameplate_<id>.png
    │  INSPECTION STATION          BAY 00    │  1.10 x 0.18 m, station accent
    └────────────────────────────────────────┘
    ┌────────────────────────────────────────┐  marker_<id>.png
    │                                        │  1.10 x 1.10 m, matte white
    │            ┌──────────────┐            │
    │            │  ArUco 4x4   │            │  0.70 m marker, DICT_4X4_50
    │            └──────────────┘            │  0.20 m (1.7 cell) quiet zone
    │                                        │
    └────────────────────────────────────────┘
    ┌────────────────────────────────────────┐  infoplate_<id>.png
    │ DICT_4X4_50 · ID 0 · 700 mm · TASK …   │  1.10 x 0.14 m, charcoal
    └────────────────────────────────────────┘

Why the text lives on separate plates instead of on the board
─────────────────────────────────────────────────────────────
OpenCV's ArUco contour stage becomes unreliable when the white carrier board
touches the image border: candidate quads that run off-frame get discarded, and
detection drops out entirely even though the marker itself is fully visible and
several hundred pixels across.  Keeping the board free of furniture lets it stay
small enough to sit wholly inside the frame at the berth, while the nameplate and
info plate — which may safely clip — carry the human-readable branding.

Detection budget (see docs/architecture.md § "Marker legibility budget")
────────────────────────────────────────────────────────────────────────
Mast camera: 640x480, 60° HFOV, optical centre 0.15 m ahead of the robot datum
and level with the board centre at 0.85 m.  Berth standoff is 1.75 m, so the
board is imaged from 1.60 m:

    board on frame   1.10 m → 381 px   (must stay under ~460 px, i.e. d ≥ 1.35 m)
    marker edge      0.70 m → 242 px   → perimeter 970 → confidence 0.81

``marker_detector_node`` scores confidence as ``perimeter / 1200`` and the
registry gate is 0.60, giving an acceptance window of roughly 1.50 m – 2.30 m
from the board.  The berth sits in the middle of that window, and the far edge
is what stops the robot dispatching a bay it is only driving past.

Usage:
    python3 scripts/generate_marker_boards.py [--output-dir models/materials/textures]
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# ── Physical geometry — MUST stay in sync with worlds/industrial_station_world.sdf
BOARD_M = 1.10          # white carrier board edge, m
MARKER_M = 0.70         # ArUco marker edge, m
NAMEPLATE_M = (1.10, 0.18)
INFOPLATE_M = (1.10, 0.14)
STANDOFF_M = 1.65       # robot datum → board plane at the berth
CAM_FORWARD_M = 0.15    # optical centre ahead of the robot datum
CAM_HFOV_RAD = 1.047
CAM_W, CAM_H = 640, 480

# ── Texture resolution ────────────────────────────────────────────────────────
BOARD_TEX = 1024
MARKER_PX = 648         # 1024 * 0.70/1.10, rounded down to a multiple of 6 cells
PLATE_W = 1024

# ── Station branding ──────────────────────────────────────────────────────────
STATIONS = {
    0: {"name": "INSPECTION", "task": "INSPECTION", "accent": (0x1F, 0x6F, 0xB2)},
    1: {"name": "ASSEMBLY",   "task": "ASSEMBLY",   "accent": (0xB5, 0x6A, 0x14)},
    2: {"name": "STORAGE",    "task": "STORAGE",    "accent": (0x2F, 0x7D, 0x4F)},
    3: {"name": "CHARGING",   "task": "CHARGING",   "accent": (0x6B, 0x3F, 0xA0)},
}

BOARD_WHITE = (0xFA, 0xFA, 0xF8)
CHARCOAL = (0x22, 0x26, 0x2B)
PLATE_TEXT = (0xFF, 0xFF, 0xFF)
INFO_TEXT = (0xE8, 0xEC, 0xF0)
INFO_SUB = (0x93, 0xA0, 0xAD)

FONTS_BOLD = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSansBold.ttf",
]
FONTS_REG = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    "/usr/share/fonts/truetype/freefont/FreeSans.ttf",
]


def _font(candidates, size):
    for path in candidates:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def _fit_font(draw, text, candidates, max_w, max_h, start=110, floor=14):
    for size in range(start, floor - 1, -2):
        f = _font(candidates, size)
        l, t, r, b = draw.textbbox((0, 0), text, font=f)
        if (r - l) <= max_w and (b - t) <= max_h:
            return f
    return _font(candidates, floor)


def _text(draw, xy, text, font, fill, align="left"):
    l, t, r, b = draw.textbbox((0, 0), text, font=font)
    x, y = xy
    if align == "center":
        x -= (r - l) / 2
    elif align == "right":
        x -= (r - l)
    draw.text((x - l, y - (b - t) / 2 - t), text, font=font, fill=fill)


def aruco_marker(marker_id: int, side_px: int) -> np.ndarray:
    """Render a DICT_4X4_50 marker with crisp, nearest-neighbour cell edges."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    base = cv2.aruco.generateImageMarker(dictionary, marker_id, 6, borderBits=1)
    return cv2.resize(base, (side_px, side_px), interpolation=cv2.INTER_NEAREST)


def build_board(marker_id: int) -> Image.Image:
    """Matte-white carrier board: marker only, nothing inside the quiet zone."""
    board = Image.new("RGB", (BOARD_TEX, BOARD_TEX), BOARD_WHITE)
    marker = aruco_marker(marker_id, MARKER_PX)
    origin = (BOARD_TEX - MARKER_PX) // 2
    board.paste(Image.fromarray(cv2.cvtColor(marker, cv2.COLOR_GRAY2RGB)), (origin, origin))

    quiet_cells = origin / (MARKER_PX / 6.0)
    assert quiet_cells >= 1.5, f"quiet zone only {quiet_cells:.2f} cells"
    return board


def build_nameplate(marker_id: int) -> Image.Image:
    meta = STATIONS[marker_id]
    h = int(round(PLATE_W * NAMEPLATE_M[1] / NAMEPLATE_M[0]))
    img = Image.new("RGB", (PLATE_W, h), meta["accent"])
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, h - 6, PLATE_W, h], fill=CHARCOAL)

    pad = 36
    bay = f"BAY {marker_id:02d}"
    bay_font = _fit_font(draw, bay, FONTS_BOLD, max_w=int(PLATE_W * 0.26),
                         max_h=int(h * 0.46), start=int(h * 0.46))
    bay_w = draw.textbbox((0, 0), bay, font=bay_font)[2]

    title = f"{meta['name']} STATION"
    title_font = _fit_font(draw, title, FONTS_BOLD,
                           max_w=PLATE_W - bay_w - 3 * pad, max_h=int(h * 0.52),
                           start=int(h * 0.52))

    _text(draw, (pad, h * 0.46), title, title_font, PLATE_TEXT)
    _text(draw, (PLATE_W - pad, h * 0.46), bay, bay_font, PLATE_TEXT, align="right")
    return img


def build_infoplate(marker_id: int) -> Image.Image:
    meta = STATIONS[marker_id]
    h = int(round(PLATE_W * INFOPLATE_M[1] / INFOPLATE_M[0]))
    img = Image.new("RGB", (PLATE_W, h), CHARCOAL)
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, 0, PLATE_W, 5], fill=meta["accent"])

    pad = 36
    right = f"TASK: {meta['task']}"
    right_font = _fit_font(draw, right, FONTS_REG, max_w=int(PLATE_W * 0.30),
                           max_h=int(h * 0.34), start=int(h * 0.34))
    right_w = draw.textbbox((0, 0), right, font=right_font)[2]
    left_budget = PLATE_W - right_w - 3 * pad

    line1 = f"ArUco DICT_4X4_50   ·   ID {marker_id}   ·   {int(round(MARKER_M * 1000))} mm"
    line1_font = _fit_font(draw, line1, FONTS_BOLD, max_w=left_budget,
                           max_h=int(h * 0.34), start=int(h * 0.34))
    line2 = "SECURE STATION DISPATCH  ·  AMR FIDUCIAL BEACON"
    line2_font = _fit_font(draw, line2, FONTS_REG, max_w=left_budget,
                           max_h=int(h * 0.24), start=int(h * 0.24))

    _text(draw, (pad, h * 0.42), line1, line1_font, INFO_TEXT)
    _text(draw, (pad, h * 0.76), line2, line2_font, INFO_SUB)
    _text(draw, (PLATE_W - pad, h * 0.56), right, right_font, INFO_SUB, align="right")
    return img


# ── Verification ──────────────────────────────────────────────────────────────
def _render_view(board_path: str, nameplate_path: str, infoplate_path: str, d_cam: float):
    """Project the whole sign stack through the mast-camera pinhole model."""
    px_per_m = (CAM_W / 2.0) / (d_cam * math.tan(CAM_HFOV_RAD / 2.0))
    canvas = np.full((CAM_H, CAM_W, 3), 198, np.uint8)

    def paste(img_path, width_m, height_m, centre_z_m):
        img = cv2.imread(img_path)
        w = max(1, int(round(width_m * px_per_m)))
        h = max(1, int(round(height_m * px_per_m)))
        scaled = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
        x0 = int(round(CAM_W / 2.0 - w / 2.0))
        y0 = int(round(CAM_H / 2.0 - centre_z_m * px_per_m - h / 2.0))
        xs0, xs1 = max(0, x0), min(CAM_W, x0 + w)
        ys0, ys1 = max(0, y0), min(CAM_H, y0 + h)
        if xs0 < xs1 and ys0 < ys1:
            canvas[ys0:ys1, xs0:xs1] = scaled[ys0 - y0:ys1 - y0, xs0 - x0:xs1 - x0]

    gap = 0.05
    paste(board_path, BOARD_M, BOARD_M, 0.0)
    paste(nameplate_path, *NAMEPLATE_M, BOARD_M / 2 + gap + NAMEPLATE_M[1] / 2)
    paste(infoplate_path, *INFOPLATE_M, -(BOARD_M / 2 + gap + INFOPLATE_M[1] / 2))
    return canvas


def _score(canvas, marker_id):
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters(),
    )
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(canvas, cv2.COLOR_BGR2GRAY))
    found = [] if ids is None else ids.flatten().tolist()
    if marker_id not in found:
        return None, 0.0
    per = cv2.arcLength(corners[found.index(marker_id)][0], True)
    return per, min(per / 1200.0, 1.0)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Generate station signage textures.")
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    parser.add_argument("--output-dir", default=os.path.join(root, "models", "materials", "textures"))
    parser.add_argument("--gate", type=float, default=0.75, help="registry min_confidence")
    parser.add_argument("--dump-view", default="", help="write the berth camera view to this path")
    args = parser.parse_args(argv)
    os.makedirs(args.output_dir, exist_ok=True)

    paths = {}
    for marker_id in sorted(STATIONS):
        b = os.path.join(args.output_dir, f"marker_{marker_id}.png")
        n = os.path.join(args.output_dir, f"nameplate_{marker_id}.png")
        i = os.path.join(args.output_dir, f"infoplate_{marker_id}.png")
        build_board(marker_id).save(b, optimize=True)
        build_nameplate(marker_id).save(n, optimize=True)
        build_infoplate(marker_id).save(i, optimize=True)
        paths[marker_id] = (b, n, i)
        print(f"  bay {marker_id}: board + nameplate + infoplate → {os.path.basename(b)}")

    print(f"\n  Board geometry: {BOARD_M:.2f} m board · {MARKER_M:.2f} m marker "
          f"· quiet zone {(BOARD_TEX - MARKER_PX) / 2 / (MARKER_PX / 6):.2f} cells")

    print("\n  Round-trip verification (raw texture → detector):")
    failures = []
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters(),
    )
    for marker_id, (b, _n, _i) in paths.items():
        _, ids, _ = detector.detectMarkers(cv2.cvtColor(cv2.imread(b), cv2.COLOR_BGR2GRAY))
        found = [] if ids is None else ids.flatten().tolist()
        ok = found == [marker_id]
        print(f"    board {marker_id}: detected {found}  [{'ok' if ok else 'FAIL'}]")
        if not ok:
            failures.append(("roundtrip", marker_id))

    d_berth = STANDOFF_M - CAM_FORWARD_M
    print(f"\n  Simulated mast-camera sweep ({CAM_W}x{CAM_H}, 60° HFOV, "
          f"berth range {d_berth:.2f} m, gate {args.gate:.2f}):")
    print(f"    {'range':>7} {'standoff':>9} {'perimeter':>10} {'conf':>6}   verdict")
    for d in [1.10, 1.25, 1.35, 1.45, d_berth, 1.75, 1.95, 2.15, 2.35, 2.80, 3.50]:
        per, conf = _score(_render_view(*paths[0], d), 0)
        verdict = "ACCEPT" if conf >= args.gate else "reject"
        tag = "  ← berth" if abs(d - d_berth) < 1e-6 else ""
        print(f"    {d:>6.2f}m {d + CAM_FORWARD_M:>8.2f}m {(f'{per:.0f}' if per else '—'):>10} "
              f"{conf:>6.2f}   {verdict}{tag}")

    per, conf = _score(_render_view(*paths[0], d_berth), 0)
    if conf < args.gate:
        print(f"\nERROR: berth confidence {conf:.2f} is below the {args.gate:.2f} gate", file=sys.stderr)
        failures.append(("berth", 0))

    if args.dump_view:
        cv2.imwrite(args.dump_view, _render_view(*paths[0], d_berth))
        print(f"\n  berth camera view written to {args.dump_view}")

    if failures:
        print(f"\nERROR: {failures}", file=sys.stderr)
        return 1
    print("\nAll station signage generated and verified.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
