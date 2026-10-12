#!/usr/bin/env python3
"""
Assistive Handwriting System - Vision Tracking Node
===================================================

Tracks a colour-marked pen tip with an overhead camera, measures its distance to
the calibrated guide line(s) (the grooves), and tells the ESP32 to vibrate the
haptic pen when the writer drifts off-track or leaves the page area.

Pipeline per frame:
    camera -> HSV colour mask -> largest blob centroid -> EMA smoothing
           -> distance to nearest guide polyline -> debounce -> "VIB <int> <ms>"

Transport to the ESP32:  USB-Serial (--serial PORT)  or  BLE (--ble [NAME|ADDR]).
Use --dry-run to test the vision side without hardware.

Run:  python tracker.py --serial /dev/ttyUSB0
      python tracker.py --ble
      python tracker.py --dry-run --video sample.mp4
Press 'c' in the window to calibrate (guide line, page bounds, pen colour).
"""

from __future__ import annotations

import argparse
import asyncio
import json
import queue
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path
from typing import Callable, Optional

import cv2
import numpy as np

try:
    import serial  # pyserial
except ImportError:  # only needed for --serial
    serial = None
try:
    from bleak import BleakClient, BleakScanner
except ImportError:  # only needed for --ble
    BleakClient = BleakScanner = None

CONFIG_PATH = Path(__file__).with_name("calibration.json")

# Nordic UART Service (must match the firmware)
NUS_RX_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   # write  (PC -> ESP32)
NUS_TX_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   # notify (ESP32 -> PC)


# ----------------------------------------------------------------------------
# Configuration (persisted to calibration.json)
# ----------------------------------------------------------------------------
@dataclass
class Config:
    # pen tip marker colour range in HSV (OpenCV: H 0-179, S/V 0-255). Default: green.
    hsv_lower: list = field(default_factory=lambda: [40, 80, 80])
    hsv_upper: list = field(default_factory=lambda: [80, 255, 255])
    min_blob_area: float = 25.0        # px^2, ignore smaller blobs (noise)
    smoothing: float = 0.5             # EMA factor, 1.0 = no smoothing

    guide_lines: list = field(default_factory=list)   # [[ [x,y], ... ], ...]
    page_bounds: Optional[list] = None                # [x1, y1, x2, y2]

    tolerance_px: float = 14.0         # allowed distance from groove centre
    debounce_frames: int = 3           # consecutive bad frames before alert
    lost_frames: int = 6               # frames without pen => treat as lifted
    min_intensity: int = 90            # vibration at the tolerance edge
    resend_ms: int = 80                # re-send period while deviating
    oob_intensity: int = 255           # out-of-bounds strength

    # automatic page turn
    auto_page_turn: bool = False
    complete_progress: float = 0.95    # fraction of the last guide line written
    page_idle_s: float = 2.5           # pen away this long => page is done
    page_busy_timeout_s: float = 30.0

    @classmethod
    def load(cls, path: Path) -> "Config":
        cfg = cls()
        if path.exists():
            data = json.loads(path.read_text())
            known = {f.name for f in fields(cls)}
            for k, v in data.items():
                if k in known:
                    setattr(cfg, k, v)
        return cfg

    def save(self, path: Path) -> None:
        path.write_text(json.dumps(asdict(self), indent=2))


# ----------------------------------------------------------------------------
# ESP32 links
# ----------------------------------------------------------------------------
class Link:
    """Abstract line-based command link. Incoming lines go to on_message."""

    def __init__(self) -> None:
        self.on_message: Callable[[str], None] = lambda _m: None

    def send(self, line: str) -> None: ...
    def close(self) -> None: ...
    @property
    def connected(self) -> bool: return True


class DryRunLink(Link):
    def send(self, line: str) -> None:
        print(f"[dry-run] -> {line}")


class SerialLink(Link):
    def __init__(self, port: str, baud: int = 115200) -> None:
        super().__init__()
        if serial is None:
            raise SystemExit("pyserial not installed: pip install pyserial")
        self._ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2.0)                       # ESP32 auto-resets on port open
        self._lock = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._reader, daemon=True).start()

    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                raw = self._ser.readline()
            except Exception:
                break
            if raw:
                msg = raw.decode(errors="ignore").strip()
                if msg:
                    self.on_message(msg)

    def send(self, line: str) -> None:
        with self._lock:
            try:
                self._ser.write((line + "\n").encode())
            except Exception as e:
                print(f"[serial] write failed: {e}")

    def close(self) -> None:
        self._stop.set()
        try:
            self.send("VIB_OFF")
            self._ser.close()
        except Exception:
            pass


class BLELink(Link):
    """Nordic-UART BLE client running its own asyncio loop in a thread."""

    def __init__(self, target: str = "AssistivePen-ESP32") -> None:
        super().__init__()
        if BleakClient is None:
            raise SystemExit("bleak not installed: pip install bleak")
        self._target = target
        self._q: "queue.Queue[str]" = queue.Queue(maxsize=64)
        self._stop = threading.Event()
        self._connected = False
        threading.Thread(target=lambda: asyncio.run(self._main()), daemon=True).start()

    @property
    def connected(self) -> bool:
        return self._connected

    def send(self, line: str) -> None:
        try:
            self._q.put_nowait(line)
        except queue.Full:
            pass                              # drop: stale haptic commands are useless

    def close(self) -> None:
        self.send("VIB_OFF")
        time.sleep(0.2)
        self._stop.set()

    async def _main(self) -> None:
        while not self._stop.is_set():
            try:
                print(f"[ble] looking for '{self._target}' ...")
                if ":" in self._target or "-" in self._target and len(self._target) > 20:
                    dev = self._target                      # address / UUID
                else:
                    dev = await BleakScanner.find_device_by_name(self._target, timeout=10)
                if dev is None:
                    continue
                async with BleakClient(dev) as client:
                    await client.start_notify(
                        NUS_TX_UUID,
                        lambda _c, data: [self.on_message(m.strip())
                                          for m in data.decode(errors="ignore").splitlines() if m.strip()],
                    )
                    self._connected = True
                    print("[ble] connected")
                    while client.is_connected and not self._stop.is_set():
                        try:
                            msg = self._q.get_nowait()
                        except queue.Empty:
                            await asyncio.sleep(0.005)
                            continue
                        await client.write_gatt_char(NUS_RX_UUID, (msg + "\n").encode(), response=False)
            except Exception as e:
                print(f"[ble] {e}")
            finally:
                self._connected = False
            await asyncio.sleep(1.5)


# ----------------------------------------------------------------------------
# Vision helpers
# ----------------------------------------------------------------------------
def closest_on_polyline(p: np.ndarray, pts: np.ndarray) -> tuple[float, float]:
    """Return (distance, progress 0..1 along the polyline) for point p."""
    seg = np.diff(pts, axis=0)
    seg_len = np.linalg.norm(seg, axis=1)
    total = float(seg_len.sum())
    best_d, best_prog, acc = float("inf"), 0.0, 0.0
    for a, ab, length in zip(pts[:-1], seg, seg_len):
        l2 = float(ab @ ab)
        t = 0.0 if l2 == 0 else float(np.clip(((p - a) @ ab) / l2, 0.0, 1.0))
        d = float(np.linalg.norm(p - (a + t * ab)))
        if d < best_d:
            best_d = d
            best_prog = (acc + t * length) / total if total > 0 else 0.0
        acc += float(length)
    return best_d, best_prog


class PenDetector:
    """Finds the coloured pen-tip marker via HSV thresholding."""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self._smoothed: Optional[np.ndarray] = None
        self.mask: Optional[np.ndarray] = None

    def detect(self, frame: np.ndarray) -> Optional[np.ndarray]:
        hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (5, 5), 0), cv2.COLOR_BGR2HSV)
        lo, hi = np.array(self.cfg.hsv_lower), np.array(self.cfg.hsv_upper)
        if lo[0] <= hi[0]:
            mask = cv2.inRange(hsv, lo, hi)
        else:                                             # hue wraps around (e.g. red)
            mask = cv2.inRange(hsv, lo, np.array([179, hi[1], hi[2]])) | \
                   cv2.inRange(hsv, np.array([0, lo[1], lo[2]]), hi)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        self.mask = mask

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._smoothed = None
            return None
        c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(c) < self.cfg.min_blob_area:
            self._smoothed = None
            return None
        (x, y), _r = cv2.minEnclosingCircle(c)
        raw = np.array([x, y], dtype=np.float64)
        a = self.cfg.smoothing
        self._smoothed = raw if self._smoothed is None else a * raw + (1 - a) * self._smoothed
        return self._smoothed.copy()


# ----------------------------------------------------------------------------
# Tracker application
# ----------------------------------------------------------------------------
class Tracker:
    WIN = "Assistive Handwriting - Tracker"

    def __init__(self, cfg: Config, link: Link, source, flip: bool, show: bool) -> None:
        self.cfg, self.link, self.flip, self.show = cfg, link, flip, show
        self.detector = PenDetector(cfg)
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise SystemExit(f"Cannot open video source: {source}")
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.trail: deque = deque(maxlen=300)
        self.state = "INIT"
        self.dist = 0.0
        self.paused = False

        # haptic bookkeeping
        self._bad_frames = 0
        self._lost = 0
        self._vib_on = False
        self._last_tx = 0.0

        # page-turn bookkeeping
        self._max_progress = 0.0
        self._last_seen = time.time()
        self._esp_busy_since: Optional[float] = None

        # calibration UI
        self.calibrating = False
        self.mode = "line"                 # line | bounds | color
        self._bounds_pt: Optional[tuple] = None
        self._last_frame: Optional[np.ndarray] = None
        self._mask_win = False

        link.on_message = self._on_esp_message

    # ---- ESP32 feedback ----------------------------------------------------
    def _on_esp_message(self, msg: str) -> None:
        if msg.startswith("DONE"):
            self._esp_busy_since = None
            self._max_progress = 0.0
            print(f"[esp32] {msg}")
        elif msg not in ("OK", "PONG"):
            print(f"[esp32] {msg}")

    @property
    def esp_busy(self) -> bool:
        if self._esp_busy_since is None:
            return False
        if time.time() - self._esp_busy_since > self.cfg.page_busy_timeout_s:
            self._esp_busy_since = None
            return False
        return True

    # ---- haptics -----------------------------------------------------------
    def _haptic(self, intensity: int, now: float) -> None:
        if now - self._last_tx < self.cfg.resend_ms / 1000.0:
            return
        # Duration is ~2.5x the resend period: if the PC/link dies, the motor
        # stops by itself within a fraction of a second.
        self.link.send(f"VIB {int(np.clip(intensity, 1, 255))} {int(self.cfg.resend_ms * 2.5)}")
        self._last_tx, self._vib_on = now, True

    def _haptic_off(self) -> None:
        if self._vib_on:
            self.link.send("VIB_OFF")
            self._vib_on = False

    # ---- core logic --------------------------------------------------------
    def evaluate(self, pen: Optional[np.ndarray]) -> None:
        now = time.time()
        cfg = self.cfg

        if pen is None:                                    # pen lifted / not visible
            self._lost += 1
            if self._lost >= cfg.lost_frames:
                self.state = "LOST"
                self._bad_frames = 0
                self._haptic_off()
                self._maybe_auto_page_turn(now)
            return

        self._lost = 0
        self._last_seen = now
        self.trail.append(tuple(pen.astype(int)))

        # 1) page bounds
        if cfg.page_bounds:
            x1, y1, x2, y2 = cfg.page_bounds
            if not (x1 <= pen[0] <= x2 and y1 <= pen[1] <= y2):
                self.state = "OUT_OF_BOUNDS"
                self._haptic(cfg.oob_intensity, now)
                return

        # 2) distance to the nearest guide line
        best_d, best_prog, best_idx = float("inf"), 0.0, -1
        for i, line in enumerate(cfg.guide_lines):
            if len(line) < 2:
                continue
            d, prog = closest_on_polyline(pen, np.array(line, dtype=np.float64))
            if d < best_d:
                best_d, best_prog, best_idx = d, prog, i
        if best_idx < 0:
            self.state = "NO_GUIDE"
            return
        self.dist = best_d

        # progress on the final line decides when the page is complete
        if best_idx == len(cfg.guide_lines) - 1 and best_d < cfg.tolerance_px * 2:
            self._max_progress = max(self._max_progress, best_prog)

        # 3) debounce + proportional intensity
        if best_d > cfg.tolerance_px:
            self._bad_frames += 1
            if self._bad_frames >= cfg.debounce_frames:
                self.state = "DEVIATING"
                ratio = float(np.clip((best_d - cfg.tolerance_px) / (cfg.tolerance_px * 2), 0, 1))
                self._haptic(int(cfg.min_intensity + ratio * (255 - cfg.min_intensity)), now)
        else:
            self._bad_frames = 0
            self.state = "OK"
            self._haptic_off()

    def _maybe_auto_page_turn(self, now: float) -> None:
        cfg = self.cfg
        if (cfg.auto_page_turn and not self.esp_busy
                and self._max_progress >= cfg.complete_progress
                and now - self._last_seen >= cfg.page_idle_s):
            print("[tracker] page complete -> PAGE_TURN")
            self.trigger_page_turn()

    def trigger_page_turn(self) -> None:
        self._haptic_off()
        self.link.send("PAGE_TURN")
        self._esp_busy_since = time.time()
        self.trail.clear()

    # ---- drawing -----------------------------------------------------------
    def draw(self, frame: np.ndarray, pen: Optional[np.ndarray], fps: float) -> np.ndarray:
        out = frame.copy()
        cfg = self.cfg
        band = out.copy()
        for line in cfg.guide_lines:
            if len(line) >= 2:
                pts = np.array(line, dtype=np.int32)
                cv2.polylines(band, [pts], False, (255, 200, 0), int(cfg.tolerance_px * 2))
        out = cv2.addWeighted(band, 0.25, out, 0.75, 0)
        for line in cfg.guide_lines:
            for p in line:
                cv2.circle(out, tuple(p), 3, (255, 255, 0), -1)
            if len(line) >= 2:
                cv2.polylines(out, [np.array(line, dtype=np.int32)], False, (255, 255, 0), 1)
        if cfg.page_bounds:
            x1, y1, x2, y2 = cfg.page_bounds
            cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 255), 2)
        if len(self.trail) > 1:
            cv2.polylines(out, [np.array(self.trail, dtype=np.int32)], False, (255, 0, 255), 2)
        if pen is not None:
            colour = (0, 200, 0) if self.state == "OK" else (0, 0, 255)
            cv2.circle(out, tuple(pen.astype(int)), 8, colour, 2)

        lines = [f"state: {self.state}   dist: {self.dist:5.1f}px (tol {cfg.tolerance_px:.0f})",
                 f"fps: {fps:4.1f}   link: {'up' if self.link.connected else 'DOWN'}   "
                 f"progress: {self._max_progress*100:3.0f}%   auto-page: {'on' if cfg.auto_page_turn else 'off'}"]
        if self.calibrating:
            lines.append(f"CALIBRATION [{self.mode}]  g=line b=bounds k=colour n=new line "
                         f"u=undo x=clear s=save c=exit")
        else:
            lines.append("c=calibrate p=page o/f=lift open/close r=reset v=test vib a=auto-page space=pause q=quit")
        for i, t in enumerate(lines):
            cv2.putText(out, t, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
            cv2.putText(out, t, (10, 22 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return out

    # ---- calibration UI ----------------------------------------------------
    def on_mouse(self, event, x, y, _flags, _param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN or not self.calibrating:
            return
        if self.mode == "line":
            if not self.cfg.guide_lines:
                self.cfg.guide_lines.append([])
            self.cfg.guide_lines[-1].append([x, y])
        elif self.mode == "bounds":
            if self._bounds_pt is None:
                self._bounds_pt = (x, y)
            else:
                (ax, ay), (bx, by) = self._bounds_pt, (x, y)
                self.cfg.page_bounds = [min(ax, bx), min(ay, by), max(ax, bx), max(ay, by)]
                self._bounds_pt = None
        elif self.mode == "color" and self._last_frame is not None:
            patch = self._last_frame[max(0, y - 2):y + 3, max(0, x - 2):x + 3]
            h, s, v = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3).mean(axis=0)
            self.cfg.hsv_lower = [int((h - 10) % 180), int(max(40, s - 70)), int(max(40, v - 70))]
            self.cfg.hsv_upper = [int((h + 10) % 180), 255, 255]
            print(f"[calib] pen colour HSV ~({h:.0f},{s:.0f},{v:.0f}) -> "
                  f"{self.cfg.hsv_lower}..{self.cfg.hsv_upper}")

    def handle_key(self, key: int) -> bool:
        """Returns False to quit."""
        if key in (ord("q"), 27):
            return False
        ch = chr(key) if 0 <= key < 128 else ""
        if ch == "c":
            self.calibrating = not self.calibrating
            self.paused = self.calibrating
            self._haptic_off()
        elif ch == "s":
            self.cfg.save(CONFIG_PATH)
            print(f"[calib] saved to {CONFIG_PATH}")
        elif self.calibrating:
            if ch == "g": self.mode = "line"
            elif ch == "b": self.mode, self._bounds_pt = "bounds", None
            elif ch == "k": self.mode = "color"
            elif ch == "n" and self.cfg.guide_lines and self.cfg.guide_lines[-1]:
                self.cfg.guide_lines.append([])
            elif ch == "u" and self.cfg.guide_lines:
                if self.cfg.guide_lines[-1]:
                    self.cfg.guide_lines[-1].pop()
                if not self.cfg.guide_lines[-1]:
                    self.cfg.guide_lines.pop()
            elif ch == "x":
                self.cfg.guide_lines.clear(); self.cfg.page_bounds = None
        else:
            if ch == "p": self.trigger_page_turn()
            elif ch == "o": self.link.send("LIFT_OPEN")
            elif ch == "f": self.link.send("LIFT_CLOSE")
            elif ch == "r": self.link.send("RESET_OVERLAY")
            elif ch == "v": self.link.send("VIB 200 400")
            elif ch == "a":
                self.cfg.auto_page_turn = not self.cfg.auto_page_turn
            elif ch == " ":
                self.paused = not self.paused
                self._haptic_off()
        return True

    # ---- main loop ---------------------------------------------------------
    def run(self) -> None:
        if self.show:
            cv2.namedWindow(self.WIN)
            cv2.setMouseCallback(self.WIN, self.on_mouse)
        self.link.send("PING")
        t_prev, fps, frame = time.time(), 0.0, None
        try:
            while True:
                if not self.paused or frame is None:
                    ok, frame = self.cap.read()
                    if not ok:
                        print("[tracker] end of stream / camera lost")
                        break
                    if self.flip:
                        frame = cv2.flip(frame, -1)       # camera mounted upside-down
                    self._last_frame = frame
                    pen = self.detector.detect(frame)
                    if not self.paused:
                        self.evaluate(pen)
                else:
                    pen = self.detector.detect(frame)

                now = time.time()
                fps = 0.9 * fps + 0.1 / max(now - t_prev, 1e-6)
                t_prev = now

                if self.show:
                    cv2.imshow(self.WIN, self.draw(frame, pen, fps))
                    if self.calibrating and self.detector.mask is not None:
                        cv2.imshow("pen mask", self.detector.mask)
                        self._mask_win = True
                    elif self._mask_win:
                        cv2.destroyWindow("pen mask")
                        self._mask_win = False
                    if not self.handle_key(cv2.waitKey(1) & 0xFF):
                        break
                elif cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            self.link.send("VIB_OFF")
            self.link.close()
            self.cap.release()
            cv2.destroyAllWindows()


# ----------------------------------------------------------------------------
def main() -> None:
    global CONFIG_PATH
    ap = argparse.ArgumentParser(description="Assistive handwriting vision tracker")
    tx = ap.add_mutually_exclusive_group()
    tx.add_argument("--serial", metavar="PORT", help="serial port, e.g. /dev/ttyUSB0 or COM5")
    tx.add_argument("--ble", nargs="?", const="AssistivePen-ESP32", metavar="NAME|ADDR",
                    help="use BLE (default name AssistivePen-ESP32)")
    tx.add_argument("--dry-run", action="store_true", help="print commands instead of sending")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--camera", type=int, default=0, help="camera index")
    ap.add_argument("--video", help="video file instead of a camera (for testing)")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--flip", action="store_true", help="rotate image 180 degrees")
    ap.add_argument("--headless", action="store_true", help="no GUI window")
    ap.add_argument("--auto-page", action="store_true", help="enable automatic page turning")
    ap.add_argument("--config", type=Path, default=CONFIG_PATH)
    args = ap.parse_args()

    CONFIG_PATH = args.config
    cfg = Config.load(CONFIG_PATH)
    if args.auto_page:
        cfg.auto_page_turn = True

    if args.serial:
        link: Link = SerialLink(args.serial, args.baud)
    elif args.ble:
        link = BLELink(args.ble)
    else:
        link = DryRunLink()
        if not args.dry_run:
            print("No --serial/--ble given: running in dry-run mode.")

    source = args.video if args.video else args.camera
    tracker = Tracker(cfg, link, source, args.flip, show=not args.headless)
    if not args.video:
        tracker.cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        tracker.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cfg.guide_lines:
        print("No guide line calibrated yet: press 'c' then click along the groove, 's' to save.")
    tracker.run()


if __name__ == "__main__":
    main()
