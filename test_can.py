#!/usr/bin/env python3
"""Standalone CAN test for the CubeMars driver (UCAN / gs_usb on Windows).

Connects to the motors, verifies communication, and runs a small position
test. No Blender required.

Usage:
    pip install python-can gs_usb
    python test_can.py

Prerequisites:
    - UCAN adapter plugged in with WinUSB driver (Zadig)
    - Motors powered and wired (CAN_H, CAN_L, GND)
    - Periodic feedback enabled in the CubeMars app
"""

import sys
import time

sys.path.insert(0, ".")

from cubemars_driver import (
    CubeMarsDriver,
    N_MAX_MOTORS,
    _CAN_AVAILABLE,
    error_name,
    make_can_id,
    FEEDBACK_STATUS,
    decode_feedback,
)
try:
    import usb.core
    import usb.util
    _USB_AVAILABLE = True
except ImportError:
    usb = None
    _USB_AVAILABLE = False

# -- Config (match your CubeMars app settings) -------------------------------
MOTOR_IDS = [0x68, 0x69]
SPEED_ERPM = 1000
ACCEL_ERPM_S2 = 1000
TOLERANCE_DEG = 3.0
TIMEOUT = 10.0
SEND_HZ = 50
INTERFACE = "gs_usb"
CHANNEL = 0
BITRATE = 1_000_000


def banner(text: str) -> None:
    print(f"\n{'=' * 60}\n {text}\n{'=' * 60}")


def check_can_available() -> None:
    if not _CAN_AVAILABLE:
        print("[FAIL] python-can is not installed.")
        print("       Run: pip install python-can gs_usb")
        sys.exit(1)
    print("[OK] python-can is available")


def test_bus_open() -> None:
    """Verify USB device is visible and CAN bus opens."""
    banner("Test 1: Detect adapter + open CAN bus")

    # Step 1a: Scan USB for the adapter.
    if _USB_AVAILABLE:
        print("  Scanning USB devices...")
        found = False
        for dev in usb.core.find(find_all=True):
            vid = dev.idVendor
            pid = dev.idProduct
            mfr = ""
            prod = ""
            try:
                mfr = dev.manufacturer or ""
            except (usb.USBError, Exception):
                pass
            try:
                prod = dev.product or ""
            except (usb.USBError, Exception):
                pass
            label = f"  VID=0x{vid:04X} PID=0x{pid:04X}"
            if mfr:
                label += f"  [{mfr}"
                if prod:
                    label += f" / {prod}"
                label += "]"
            print(label)
            # Heuristic: UCAN / WCH / OpenMoko / CandleLight
            is_can = (
                vid in (0x16C0, 0x1209, 0x048D, 0x1D50)
                or "can" in (prod or "").lower()
                or "candlelight" in (prod or "").lower()
                or "ucan" in (prod or "").lower()
                or "schneider" in (mfr or "").lower()
                or "openmoko" in (mfr or "").lower()
                or "wch" in (mfr or "").lower()
            )
            if is_can:
                found = True
                print(f"  [OK] Likely CAN adapter: {mfr or '?'} {prod or '?'}")
        if not found:
            print("\n  [WARN] No obvious CAN adapter found in USB scan.")
            print("         (pyusb may be limited on Windows without admin.)")
            print("         Continuing to try opening the bus anyway...")
    else:
        print("  [SKIP] pyusb not available — skipping USB scan")

    # Step 1b: Open the CAN bus.
    import can
    try:
        bus = can.ThreadSafeBus(interface=INTERFACE, channel=CHANNEL,
                                bitrate=BITRATE)
        print(f"\n[OK] Opened {INTERFACE} ch{CHANNEL} at {BITRATE // 1000} kbps")
        bus.shutdown()
    except Exception as e:
        print(f"\n[FAIL] Could not open bus: {e}")
        print()
        print("  Troubleshooting:")
        print("  1. Install WinUSB driver via Zadig:")
        print("     a. Download https://zadig.akeo.ie/")
        print("     b. Options > List All Devices")
        print("     c. Find your UCAN in the dropdown")
        print("     d. Select 'WinUSB' on the right")
        print("     e. Click 'Install Driver'")
        print("     f. Unplug and replug the adapter")
        print("  2. Check Device Manager: the device should show up under")
        print("     'Universal Serial Bus devices' without a warning icon")
        print("  3. Re-run this test")
        sys.exit(1)


def test_read_feedback(motor_ids: list[int], bus=None) -> bool:
    """Read feedback from each motor to verify they're alive."""
    banner("Test 2: Read feedback from motors")
    close_bus = False
    if bus is None:
        import can
        bus = can.ThreadSafeBus(interface=INTERFACE, channel=CHANNEL,
                                bitrate=BITRATE)
        close_bus = True
    try:
        for i, mid in enumerate(motor_ids, 1):
            fb_id = make_can_id(FEEDBACK_STATUS, mid)
            fb = None
            deadline = time.time() + 1.0
            while time.time() < deadline and fb is None:
                msg = bus.recv(timeout=0.05)
                if msg and msg.arbitration_id == fb_id and len(msg.data) >= 8:
                    fb = decode_feedback(list(msg.data))
            if fb:
                err_str = f" [ERR:{error_name(fb['error'])}]" if fb['error'] else ""
                print(f"[OK] J{i} (ID 0x{mid:02X}): "
                      f"{fb['position']:7.1f} deg | "
                      f"{fb['speed']:6.0f} ERPM | "
                      f"{fb['current']:5.2f} A | "
                      f"{fb['temperature']:3d} C{err_str}")
            else:
                print(f"[FAIL] J{i} (ID 0x{mid:02X}): no feedback received")
                print(f"       Check: ID correct? Powered? CAN_H/L swapped?")
                return False
        return True
    finally:
        if close_bus:
            bus.shutdown()



def test_position_move(motor_ids: list[int], bus=None) -> bool:
    """Send a small position move to each motor and verify arrival."""
    banner("Test 3: Position move (small angles)")
    driver = CubeMarsDriver(
        interface=INTERFACE,
        channel=CHANNEL,
        bitrate=BITRATE,
        motor_ids=motor_ids + [0] * (N_MAX_MOTORS - len(motor_ids)),
        bus=bus,
    )

    targets = [30.0] * len(motor_ids) + [0.0] * (N_MAX_MOTORS - len(motor_ids))
    print(f"Moving to: {[f'{t:.1f} deg' for t in targets[:len(motor_ids)]]}")
    print(f"Speed: {SPEED_ERPM} ERPM, Accel: {ACCEL_ERPM_S2} ERPM/s^2")
    print(f"Streaming at {SEND_HZ} Hz...\n")

    try:
        driver.stream_to_targets(
            targets_deg=targets,
            speed_erpm=SPEED_ERPM,
            accel_erpm_s2=ACCEL_ERPM_S2,
            send_hz=SEND_HZ,
            tolerance_deg=TOLERANCE_DEG,
            timeout=TIMEOUT,
        )
    except RuntimeError as e:
        print(f"[FAIL] {e}")
        return False

    while driver.is_active:
        time.sleep(0.1)
        print(f"  {driver.status}")

    result = driver.status
    print(f"\nResult: {result}")
    ok = result.startswith("OK")
    if ok:
        print("[PASS] All motors reached target")
    else:
        print("[FAIL] Move did not complete successfully")

    # Move back to 0.
    print("\n--- Returning to origin (0 deg) ---")
    driver.stream_to_targets(
        targets_deg=[0.0] * N_MAX_MOTORS,
        speed_erpm=SPEED_ERPM,
        accel_erpm_s2=ACCEL_ERPM_S2,
        send_hz=SEND_HZ,
        tolerance_deg=TOLERANCE_DEG,
        timeout=TIMEOUT,
    )
    while driver.is_active:
        time.sleep(0.1)
        print(f"  {driver.status}")
    print(f"\nReturn result: {driver.status}")

    driver.close()
    return ok


def main() -> None:
    banner("CubeMars CAN Driver Test")
    print(f"  Interface: {INTERFACE}, channel {CHANNEL}, {BITRATE // 1000} kbps")
    print(f"  Motors: {', '.join(f'J{i+1}=0x{m:02X}' for i, m in enumerate(MOTOR_IDS))}")
    print(f"  Speed: {SPEED_ERPM} ERPM, Accel: {ACCEL_ERPM_S2} ERPM/s^2")
    print()
    print("WARNING: Motors will move. Secure the hardware before running.")
    try:
        input("Press Enter to start (or Ctrl+C to abort)...")
    except (KeyboardInterrupt, EOFError):
        print("\nAborted.")
        return

    check_can_available()
    test_bus_open()

    # Open a single bus to share across all tests (WinUSB only allows one
    # exclusive handle per process on gs_usb).
    import can
    bus = can.ThreadSafeBus(interface=INTERFACE, channel=CHANNEL,
                            bitrate=BITRATE)
    print("[OK] Shared bus opened for remaining tests\n")

    try:
        alive = test_read_feedback(MOTOR_IDS, bus=bus)
        if not alive:
            print("\n[ABORT] Motors not responding. Fix issues above and retry.")
            return

        ok = test_position_move(MOTOR_IDS, bus=bus)
    finally:
        bus.shutdown()

    banner("DONE" if ok else "DONE (with failures)")
    print(f"\n{'PASS' if ok else 'FAIL'}")
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()