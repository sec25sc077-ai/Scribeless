#!/usr/bin/env python3
"""
Generate reproducible demo data for the Assistive Handwriting System.

Outputs (next to this script):
  guide_calibration.json        calibration matching the synthetic scene (load with --config)
  trajectories/<scenario>.csv   per-frame pen positions (frame,x,y ; empty x,y = pen not visible)
  trajectories/demo_session.csv all scenarios concatenated
  demo.mp4                      synthetic overhead video (green marker on a page with a groove)

Run:  python generate_demo_data.py
Then: python ../vision_tracking/tracker.py --dry-run --video demo.mp4 --config guide_calibration.json --auto-page
"""
import csv
import json
from pathlib import Path

import cv2
import numpy as np

OUT = Path(__file__).parent
FPS, W, H = 30, 640, 360
GUIDE = [[60, 180], [200, 175], [400, 185], [580, 180]]     # groove centre-line (px)
BOUNDS = [30, 60, 610, 300]                                  # allowed page area
TOLERANCE = 14.0

_pts = np.array(GUIDE, float)
_seg = np.linalg.norm(np.diff(_pts, axis=0), axis=1)
_cum = np.concatenate([[0], np.cumsum(_seg)])


def on_line(progress: float) -> np.ndarray:
    """Point on the guide polyline at arc-length fraction `progress` (0..1)."""
    d = np.clip(progress, 0, 1) * _cum[-1]
    i = min(np.searchsorted(_cum, d, side="right") - 1, len(_seg) - 1)
    t = (d - _cum[i]) / _seg[i]
    return _pts[i] + t * (_pts[i + 1] - _pts[i])


def scenario(p0, p1, seconds, dy=lambda u: 0.0):
    """Pen moves along the line from progress p0->p1 with a vertical offset profile dy(u), u in 0..1."""
    n = int(seconds * FPS)
    out = []
    for k in range(n):
        u = k / max(n - 1, 1)
        x, y = on_line(p0 + (p1 - p0) * u)
        out.append((round(float(x), 1), round(float(y + dy(u)), 1)))
    return out


def absent(seconds):
    return [None] * int(seconds * FPS)


SCENARIOS = {
    # Pen follows the groove within a couple of pixels of noise -> no vibration expected.
    "on_track": scenario(0.0, 0.45, 4, lambda u: 3 * np.sin(u * 20)),
    # Pen drifts 32 px off the groove and comes back -> VIB with rising intensity, then VIB_OFF.
    "drift": scenario(0.45, 0.65, 3, lambda u: 32 * np.sin(np.pi * u)),
    # Pen leaves the page area (y > 300) -> maximum-intensity VIB (out of bounds).
    "out_of_bounds": scenario(0.65, 0.75, 2, lambda u: 150 * np.sin(np.pi * u)),
    # Pen finishes the line on track.
    "finish_line": scenario(0.75, 1.0, 3, lambda u: 2 * np.sin(u * 15)),
    # Pen lifted for 4 s -> page complete -> PAGE_TURN expected once (auto-page mode).
    "pen_lifted": absent(4),
}
SESSION_ORDER = ["on_track", "drift", "out_of_bounds", "finish_line", "pen_lifted"]


def write_csv(path, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "x", "y"])
        for i, p in enumerate(rows):
            w.writerow([i, "" if p is None else p[0], "" if p is None else p[1]])


def render_video(rows, path):
    vw = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
    base = np.full((H, W, 3), 235, np.uint8)                       # paper
    cv2.rectangle(base, (BOUNDS[0], BOUNDS[1]), (BOUNDS[2], BOUNDS[3]), (215, 215, 215), 1)
    cv2.polylines(base, [_pts.astype(np.int32)], False, (150, 150, 150), 6)   # groove
    for p in rows:
        frame = base.copy()
        if p is not None:
            cv2.circle(frame, (int(p[0]), int(p[1])), 7, (0, 200, 0), -1)      # green marker (BGR)
        vw.write(frame)
    vw.release()


def main():
    (OUT / "trajectories").mkdir(exist_ok=True)
    session = []
    for name in SESSION_ORDER:
        write_csv(OUT / "trajectories" / f"{name}.csv", SCENARIOS[name])
        session += SCENARIOS[name]
    write_csv(OUT / "trajectories" / "demo_session.csv", session)

    (OUT / "guide_calibration.json").write_text(json.dumps({
        "hsv_lower": [40, 80, 80], "hsv_upper": [80, 255, 255],
        "guide_lines": [GUIDE], "page_bounds": BOUNDS,
        "tolerance_px": TOLERANCE, "auto_page_turn": True,
    }, indent=2))
    render_video(session, OUT / "demo.mp4")
    print(f"frames: {len(session)}  ({len(session)/FPS:.1f} s) -> {OUT}")


if __name__ == "__main__":
    main()
