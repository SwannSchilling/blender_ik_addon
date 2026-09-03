# CAN Bus Setup Guide — CubeMars Actuators + PickIK arm7

This guide covers connecting CubeMars AK-series actuators to the PickIK
Blender addon over a USB-to-CAN adapter. It assumes you are on **Windows**
(the most common case); Linux/macOS notes are in §7.

---

## 1. Hardware

| Component | Notes |
|-----------|-------|
| CubeMars AK-series actuator(s) | AK60, AK80, AK10, etc. |
| USB-to-CAN adapter | **UCAN** (canable.io) recommended; any `gs_usb`-compatible board works |
| CAN wiring | CAN_H → CAN_H, CAN_L → CAN_L, GND → GND. 120 Ω termination on both ends. |
| Power | 7.4–12 V to the actuators (do NOT power from the USB adapter) |

**Wiring (per motor):**
```
UCAN                Actuator
─────────────────────────────
  CAN_H  ───────────  CAN_H
  CAN_L  ───────────  CAN_L
  GND    ───────────  GND
```
Multiple motors: daisy-chain on the same CAN_H/CAN_L. Give each motor a
**unique CAN drive ID** (1–127) in the CubeMars app.

---

## 2. Software installation (Windows)

### 2.1 Python dependencies

Use the **"Install deps"** button in the PickIK panel, or:
```bash
pip install python-can gs_usb
```

For Blender, deps install into `%APPDATA%\Blender Foundation\Blender\python-deps\py3xx\`
(the addon handles this automatically).

### 2.2 libusb-1.0.dll (Windows only)

`python-can` → `gs_usb` → `pyusb` needs the native `libusb-1.0.dll`
to enumerate USB devices. **The addon ships with a 64-bit copy** in the
addon folder; `cubemars_driver.py` adds that directory to the DLL search
path automatically. No action needed for 64-bit Python (the default).

> ⚠️ 32-bit Python needs the 32-bit DLL. See §8 Troubleshooting.

---

## 3. Driver setup (WinUSB via Zadig)

The `gs_usb` protocol requires the **WinUSB** driver. One-time setup.

1. Download [Zadig](https://zadig.akeo.ie/) (v2.4+)
2. Plug in the UCAN
3. Zadig: **Options → List All Devices** → find your adapter
4. Select **WinUSB** on the right
5. Click **Install Driver**
6. **Unplug and replug**

**Verify:** Device Manager → Universal Serial Bus devices → your adapter
shows up with no yellow ⚠️. Properties → Driver Name = `winusb.inf`.

> The adapter may have a second interface (DFU/firmware). Only the CAN
> data interface (MI_00) needs WinUSB.

---

## 4. Configure the CubeMars actuators

1. Open the **CubeMars Upper Computer** app.
2. Set the **CAN drive ID** for each motor (unique, 1–127).
3. Set **CAN bus rate** to **1 Mbps**.
4. Enable **periodic feedback**.
5. Run **motor parameter identification** + **encoder calibration**.
6. Save.

---

## 5. Test the connection

### Standalone (no Blender):
```bash
cd blender_ik_addon
python test_can.py
```

### In Blender:
PickIK panel → CubeMars section → **"Check driver"** → then
**"Send positions to actuators"**.

---

## 6. Configuring the addon

| Property | Default | Description |
|----------|---------|-------------|
| Interface | `gs_usb` | python-can backend (see table below) |
| Channel | `0` | Adapter index (0 = first device) |
| J1–J7 CAN ID | `0x68`, `0x69`, rest `0` | Drive IDs; 0 = inactive |
| Max speed | 2000 ERPM | Speed limit for moves |
| Max accel | 2000 ERPM/s² | Acceleration limit |

### Supported python-can interfaces

| Interface | Hardware | Extra deps |
|-----------|----------|------------|
| `gs_usb` | UCAN, canable.io, most open USB CAN | (none — ships with addon) |
| `slcan` | WCH CH340/CH341, FTDI serial CAN | `pyserial` |
| `pcan` | Peak PCAN-USB | PEAK driver (system install) |
| `kvaser` | Kvaser CAN/KII | Kvaser driver |
| `socketcan` | Linux only | kernel `can` module |
| `vector` | Vector VN1630/VN7600 | Vector driver |

For `slcan` on Windows, set Channel to the COM port number (e.g. 3).

---

## 7. Linux / macOS

- No WinUSB/Zadig — the OS handles USB natively.
- `libusb-1.0` is a system package (`apt install libusb-1.0-0` / `brew install libusb`).
- `socketcan`: `sudo ip link set can0 up type can bitrate 1000000`
- `gs_usb` (UCAN on Linux): auto-identifies on plug.

---

## 8. Troubleshooting

### "Cannot find device 0. Devices found: 0"

Adapter not visible to gs_usb. Checklist:
1. Is it plugged in? LED should light up.
2. Run `python test_can.py` — the USB scan lists all devices.
   - Not listed → USB hardware issue (port, cable, dead adapter).
   - Listed but open fails → driver problem (run Zadig → WinUSB).
3. **libusb-1.0.dll missing** (Windows 32-bit or manual setup).
   Download [libusb](https://github.com/libusb/libusb/releases), place
   `libusb-1.0.dll` next to `python.exe` or in `System32`.

### "Access denied (insufficient permissions)"

WinUSB handle already claimed. Fix:
- Close other programs using the adapter (CubeMars app, etc.)
- Restart Blender / Python
- Unplug and replug the adapter

### Motor doesn't respond (no feedback)

- CAN ID mismatch (addon vs CubeMars app) — run `test_can.py` to verify
- CAN_H / CAN_L swapped
- No 120 Ω termination (close the jumper on the UCAN)
- Bus rate mismatch (must be 1 Mbps both sides)
- Motor not powered (needs 7.4–12 V DC separately)

### Motor moves wrong direction / offset

- Direction: invert in CubeMars app (settings → motor → reverse)
- Origin offset: the addon sends absolute joint angle (0° = rig rest pose).
  If physical 0° ≠ rig 0°, calibrate the offset (planned for v1.1).

### Blender: "python-can is not installed"

Click **"Install deps"** in the panel. It uses Blender's Python to pip
into a writable per-user directory. No restart needed.

### "gs_usb Bus was not properly shut down"

Cosmetic warning from the gs_usb library on GC. Harmless. The addon
calls `driver.close()` in `unregister()`.

---

## 9. Shipped files

| File | Purpose |
|------|---------|
| `libusb-1.0.dll` | 64-bit libusb for Windows |
| `cubemars_driver.py` | CAN protocol + `CubeMarsDriver` class |
| `test_can.py` | Standalone connection test |
| `CAN_SETUP.md` | This document |
