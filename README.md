# Assistive Handwriting System for Visually Impaired Individuals

An ESP32 + overhead-camera system that helps a visually impaired person practise handwriting
on a micro-grooved rubber overlay. A camera watches the pen tip; when the writing drifts off
the groove (or off the page) the ESP32 vibrates the pen. When the page is finished, servos lift
the overlay, turn the page, and reposition the overlay.

```
 ┌──────────────┐  frames   ┌────────────────────┐  Serial / BLE   ┌─────────────────┐
 │ Overhead cam ├──────────►│ vision_tracking/   ├────────────────►│ esp32_firmware/ │
 └──────────────┘           │ tracker.py (OpenCV)│  "VIB 180 200"  │ main.ino        │
                            └────────────────────┘  "PAGE_TURN"    └───┬────┬────┬───┘
                                                                       │    │    └─► Haptic pen motor
                                                                       │    └──────► Page-turn servo
                                                                       └───────────► Lift + positioner servos
```

```
esp32_firmware/main.ino        Servo control, haptic driver, Serial + BLE command parser
vision_tracking/tracker.py     Pen tracking, deviation detection, ESP32 link, calibration UI
vision_tracking/requirements.txt
```

## 1. Hardware & wiring

| Function                 | ESP32 pin | Notes                                                      |
|--------------------------|-----------|------------------------------------------------------------|
| Overlay lift servo       | GPIO 18   | Signal wire only; power from external 5 V                  |
| Page-turn servo          | GPIO 19   | "                                                          |
| Overlay positioner servo | GPIO 21   | "                                                          |
| Vibration motor driver   | GPIO 25   | Gate of logic-level N-MOSFET / base of NPN via 1 kΩ        |
| Status LED               | GPIO 2    | Onboard; solid = idle, blinking = sequence running         |

**Haptic pen driver** (never drive the motor straight from a GPIO):

```
3.3V/5V ──┬───────────┐
          │       [Motor]      1N4148/1N5819 flyback diode across motor
          │           │        (cathode to +V, anode to drain/collector)
          └───────────┤
                      D
GPIO25 ──[1kΩ]────── G   (2N7000 / IRLZ44N / or NPN 2N2222 + 1 kΩ base)
                      S ── GND
```

**Power:** servos (especially under load) draw amps. Use a separate 5–6 V supply (≥2 A) for
the three servos, **share its GND with the ESP32**, and add a 470–1000 µF capacitor across the servo
supply. Do not power servos from the ESP32 3V3/5V pin.

The pen's motor leads run to the ESP32 driver through a flexible cable (or use a small
BLE-capable pen module later: the command protocol stays the same).

## 2. ESP32 firmware

**Dependencies**

- Arduino IDE 2.x (or `arduino-cli` / PlatformIO) with the **esp32 by Espressif** board package (core 2.x or 3.x)
- Library: **ESP32Servo** (Library Manager → "ESP32Servo")
- BLE is built into the ESP32 core (no extra install)

**Flash**

1. Arduino IDE needs the sketch folder name to match the `.ino` name. Either rename
   `esp32_firmware/main.ino` → `esp32_firmware/esp32_firmware.ino`, or open it and accept the
   IDE's offer to move it into a `main/` folder.
2. Board: *ESP32 Dev Module*; select the port; Upload.
3. Open the Serial Monitor at **115200** baud (newline). You should see `READY`.

**Calibrate servo angles first.** Edit the `*_ANGLE` constants at the top of `main.ino`. Move
each servo safely from the serial monitor to find good values:

```
SERVO 0 45     # 0 = lift, 1 = page arm, 2 = positioner; angle 0-180
```

Set `ENABLE_BLE 0` if you only use USB.

### Command protocol (newline-terminated, same on Serial and BLE)

| Command             | Effect                                                         | Reply            |
|---------------------|----------------------------------------------------------------|------------------|
| `PING`              | Link check                                                     | `PONG`           |
| `VIB <0-255> <ms>`  | Vibrate; auto-stops after `ms` (fail-safe if the PC dies)      | `OK` / `BUSY`    |
| `VIB_OFF`           | Stop vibration                                                 | `OK`             |
| `LIFT_OPEN` / `LIFT_CLOSE` | Raise / lower the rubber overlay                        | `OK` / `BUSY`    |
| `PAGE_TURN`         | Open overlay → turn page → retract → reposition → close        | `OK`, later `DONE PAGE_TURN` |
| `RESET_OVERLAY`     | Reposition overlay only                                        | `OK`, `DONE RESET_OVERLAY` |
| `HOME`              | Abort sequence, all servos home                                | `OK`             |
| `SERVO <id> <deg>`  | Raw servo move (calibration)                                   | `OK`             |
| `STATUS`            | Positions / state                                              | `STATUS ...`     |

BLE uses the Nordic UART Service (device name `AssistivePen-ESP32`), so any BLE terminal app
(nRF Connect, Serial Bluetooth Terminal) can send these commands for testing.

## 3. Vision tracking node

**Dependencies** (Python 3.9+)

```bash
cd vision_tracking
python -m venv .venv && source .venv/bin/activate    # Windows: .venv\Scripts\activate
pip install -r requirements.txt                      # opencv-python, numpy, pyserial, bleak
```

**Camera & pen setup**

- Mount the camera directly above the page, looking straight down, fixed rigidly (calibration is in pixels).
- Put a **small, saturated-colour marker** (green/orange/pink sticker or tape) on the pen tip/top.
  It must differ from the page, overlay and the writer's hand. Even, diffuse lighting helps.

**Run**

```bash
python tracker.py --serial /dev/ttyUSB0      # Windows: --serial COM5
python tracker.py --ble                      # BLE, default device name
python tracker.py --dry-run                  # no hardware: prints commands
python tracker.py --dry-run --video test.mp4 # replay a recording
```

Useful flags: `--camera 1`, `--flip` (camera upside-down), `--auto-page`, `--headless`,
`--width/--height`, `--config path.json`.

**First-time calibration** (press `c` in the video window, then `s` to save to `calibration.json`):

| Key | Action                                                                          |
|-----|---------------------------------------------------------------------------------|
| `k` | *Colour mode:* click the pen marker → HSV range is set automatically            |
| `g` | *Line mode:* click points along the centre of the groove (polyline). `n` = start another line, `u` = undo, `x` = clear |
| `b` | *Bounds mode:* click two opposite corners of the allowed page area              |
| `s` | Save · `c` Exit calibration                                                     |

The shaded band shows the tolerance (`tolerance_px` in `calibration.json`). Outside it for
`debounce_frames` consecutive frames → vibration, with intensity rising with distance. Leaving the page
bounds triggers maximum vibration. Pen invisible (lifted) → no vibration.

**Runtime keys:** `p` page turn · `o`/`f` lift open/close · `r` reset overlay · `v` test vibration ·
`a` toggle auto page-turn · `space` pause · `q` quit.

**Auto page turn:** with `--auto-page`, once the pen has covered ≥95 % of the *last* guide line and
has been out of view for 2.5 s, the tracker sends `PAGE_TURN`, and resumes when the ESP32 replies `DONE`.
Order your guide lines top → bottom so the last one is the final line on the page.

Tunables live in `calibration.json` (`tolerance_px`, `debounce_frames`, `min_intensity`,
`resend_ms`, `complete_progress`, `page_idle_s`, ...).

## 4. Safety & fail-safes

- Every `VIB` command carries a duration; the tracker re-sends it every `resend_ms`, so the motor stops
  within ~0.2 s if the PC, camera, or link fails. BLE disconnects also stop the motor.
- Vibration is ignored while a page-turn sequence runs; mechanics refuse new commands while busy.
- Keep fingers clear of the lift/page arms; add mechanical end stops and a hardware power switch for the servo rail.
- This is an assistive research prototype, not a certified medical device.

## 5. Troubleshooting

| Symptom                         | Fix                                                                       |
|---------------------------------|---------------------------------------------------------------------------|
| ESP32 reboots when servos move  | Servo supply shared with ESP32 / insufficient current; use separate supply + capacitor |
| Servos jitter                   | Check common GND; adjust `attach(pin, 500, 2400)` pulse range for your servos |
| Pen not detected                | Use `c` → `k`, click marker; check the *pen mask* window; improve lighting |
| False deviation alerts          | Increase `tolerance_px` / `debounce_frames`; re-draw the guide line more carefully |
| Motor weak / won't start        | Raise `VIB_MIN_DUTY`; confirm driver transistor and 3.3 V → 5 V motor supply |
| `--ble` can't find device       | Make sure no other client is connected; pass the MAC/UUID: `--ble AA:BB:...` |
| Serial port busy / permission   | Close Serial Monitor; Linux: `sudo usermod -aG dialout $USER`             |

## License

MIT (add a `LICENSE` file to your repository).
