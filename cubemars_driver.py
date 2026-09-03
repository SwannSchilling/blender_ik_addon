"""CubeMars AK-series actuator driver for the PickIK arm7 Blender addon.

Manages one shared CAN bus (UCAN / gs_usb on Windows) with up to 7 actuators.
All CAN I/O happens on a single background thread so the Blender UI never blocks.

Prerequisites:
    pip install python-can gs_usb

The UCAN adapter must have the WinUSB driver installed (via Zadig).
"""

from __future__ import annotations

import struct
import threading
import time

# Graceful import: the addon loads fine without python-can; errors surface
# only when the user actually tries to send to the actuators.
try:
    import can
    _CAN_AVAILABLE = True
except ImportError:
    can = None
    _CAN_AVAILABLE = False


# ============================================================================
# PROTOCOL CONSTANTS (CubeMars AK-series, manual V3.2.0)
# ============================================================================

MODE_DUTY_CYCLE = 0
MODE_CURRENT_LOOP = 1
MODE_CURRENT_BRAKE = 2
MODE_VELOCITY_LOOP = 3
MODE_POSITION_LOOP = 4
MODE_SET_ORIGIN = 5
MODE_POSITION_VELOCITY = 6
MODE_FORCE_CONTROL_MIT = 8
MODE_MOTOR_DISABLE = 15
MODE_FEEDBACK_CONFIG = 16

FEEDBACK_START_FRAME = 0x2C
FEEDBACK_STATUS = 0x29
FEEDBACK_EXTENDED_POSITION = 0x2A

ERROR_NAMES = {
    0: "None",
    1: "Over-temperature",
    2: "Over-current",
    3: "Over-voltage",
    4: "Under-voltage",
    5: "Encoder fault",
    6: "MOSFET over-temperature",
    7: "Motor lock-up",
}


def error_name(code: int) -> str:
    return ERROR_NAMES.get(code, f"Unknown ({code})")


def make_can_id(mode_id: int, drive_id: int) -> int:
    """Construct CAN ID: (mode_id << 8) | drive_id"""
    return (mode_id << 8) | drive_id


def pack_position_velocity(pos_deg: float, speed_erpm: float,
                          accel_erpm_s2: float) -> bytes:
    """Pack Mode 6 (Position-Velocity) 8-byte payload, big-endian."""
    pos_raw = int(pos_deg * 10000.0)
    speed_raw = int(speed_erpm / 10)
    accel_raw = int(accel_erpm_s2 / 10)
    if accel_raw < 0:
        accel_raw = 0
    return struct.pack('>ihh', pos_raw, speed_raw, accel_raw)


def decode_feedback(data: list[int]) -> dict | None:
    """Decode an 8-byte feedback frame into a dict."""
    if len(data) < 8:
        return None
    pos_raw = (data[0] << 8) | data[1]
    spd_raw = (data[2] << 8) | data[3]
    cur_raw = (data[4] << 8) | data[5]
    if pos_raw > 32767:
        pos_raw -= 65536
    if spd_raw > 32767:
        spd_raw -= 65536
    if cur_raw > 32767:
        cur_raw -= 65536
    temp = data[6]
    if temp > 127:
        temp -= 256
    return {
        'position': pos_raw * 0.1,
        'speed': spd_raw * 10.0,
        'current': cur_raw * 0.01,
        'temperature': temp,
        'error': data[7],
    }


# ============================================================================
# DRIVER
# ============================================================================

N_MAX_MOTORS = 7


class CubeMarsDriver:
    """Manages one shared CAN bus with up to 7 CubeMars AK-series actuators.

    All CAN I/O happens on a single background thread so the Blender UI
    never blocks. The caller (Blender operator) calls ``stream_to_targets``
    which is non-blocking; a Blender timer polls ``status`` for updates.
    """

    def __init__(self, interface: str = "gs_usb", channel: int = 0,
                 bitrate: int = 1_000_000,
                 motor_ids: list[int] | None = None):
        if motor_ids is None:
            motor_ids = [0] * N_MAX_MOTORS
        if len(motor_ids) != N_MAX_MOTORS:
            raise ValueError(f"motor_ids must have exactly {N_MAX_MOTORS} entries")
        self._interface = interface
        self._channel = channel
        self._bitrate = bitrate
        self._motor_ids = list(motor_ids)
        self._active_idx = [i for i, m in enumerate(motor_ids) if m != 0]
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._status: str = "idle"
        self._bus = None

    @property
    def is_active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def status(self) -> str:
        with self._lock:
            return self._status

    def stream_to_targets(self, targets_deg: list[float],
                          speed_erpm: float = 2000.0,
                          accel_erpm_s2: float = 2000.0,
                          send_hz: float = 50.0,
                          tolerance_deg: float = 2.0,
                          timeout: float = 15.0) -> None:
        """Start streaming position commands to all active motors (non-blocking)."""
        if self.is_active:
            raise RuntimeError("Already streaming. Call stop() first.")
        if not _CAN_AVAILABLE:
            raise RuntimeError(
                "python-can is not installed. Run: pip install python-can gs_usb")
        if not self._active_idx:
            raise RuntimeError("No active motors configured (all IDs are 0).")
        self._stop_event.clear()
        self._set_status("opening bus...")
        self._thread = threading.Thread(
            target=self._worker,
            args=(targets_deg, speed_erpm, accel_erpm_s2,
                  send_hz, tolerance_deg, timeout),
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        """Signal the streaming thread to stop and disable all motors."""
        if self.is_active:
            self._stop_event.set()
            self._set_status("stopping...")

    def close(self) -> None:
        """Stop streaming (if active) and wait for the thread to finish."""
        self.stop()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    def _set_status(self, text: str) -> None:
        with self._lock:
            self._status = text

    def _open_bus(self) -> None:
        try:
            self._bus = can.ThreadSafeBus(
                interface=self._interface,
                channel=self._channel,
                bitrate=self._bitrate,
            )
        except Exception as e:
            raise RuntimeError(
                f"Failed to open CAN bus ({self._interface} ch{self._channel}): {e}"
            ) from e

    def _close_bus(self) -> None:
        if self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
            self._bus = None

    def _disable_all(self) -> None:
        if self._bus is None:
            return
        for idx in self._active_idx:
            drive_id = self._motor_ids[idx]
            can_id = make_can_id(MODE_MOTOR_DISABLE, drive_id)
            msg = can.Message(
                arbitration_id=can_id,
                data=[0] * 8,
                is_extended_id=True,
                dlc=8,
            )
            try:
                self._bus.send(msg)
            except Exception:
                pass

    def _worker(self, targets_deg: list[float], speed_erpm: float,
                accel_erpm_s2: float, send_hz: float,
                tolerance_deg: float, timeout: float) -> None:
        """Background thread: stream Mode 6 frames until done/stop/timeout."""
        try:
            self._open_bus()
        except RuntimeError as e:
            self._set_status(f"ERROR: {e}")
            return

        interval = 1.0 / send_hz
        start_time = time.time()
        deadline = start_time + timeout

        # Pre-build CAN messages for each active motor.
        msgs: list[tuple[int, object, int]] = []
        for idx in self._active_idx:
            drive_id = self._motor_ids[idx]
            target = targets_deg[idx]
            payload = pack_position_velocity(target, speed_erpm, accel_erpm_s2)
            can_id = make_can_id(MODE_POSITION_VELOCITY, drive_id)
            msg = can.Message(
                arbitration_id=can_id,
                data=list(payload),
                is_extended_id=True,
                dlc=len(payload),
            )
            msgs.append((idx, msg, drive_id))

        fb_ids: dict[int, int] = {
            idx: make_can_id(FEEDBACK_STATUS, drive_id)
            for idx, _, drive_id in msgs
        }

        frame_count = 0
        last_status_update = 0.0

        try:
            while not self._stop_event.is_set() and time.time() < deadline:
                tick_start = time.time()

                # 1) Send one position-velocity frame to each active motor.
                for _idx, msg, _ in msgs:
                    try:
                        self._bus.send(msg)
                    except Exception:
                        self._set_status("CAN send error")
                        return
                frame_count += 1

                # 2) Poll feedback for arrival checking.
                arrived = True
                positions: dict[int, float] = {}
                fb_deadline = time.time() + interval * 0.5

                while time.time() < fb_deadline:
                    try:
                        msg_in = self._bus.recv(timeout=0.005)
                    except Exception:
                        break
                    if msg_in is None:
                        continue
                    for idx, fb_id in fb_ids.items():
                        if (msg_in.arbitration_id == fb_id
                                and len(msg_in.data) >= 8):
                            fb = decode_feedback(list(msg_in.data))
                            if fb:
                                positions[idx] = fb['position']

                # Check arrival for all active motors.
                for idx, _, _ in msgs:
                    if idx in positions:
                        if abs(positions[idx] - targets_deg[idx]) > tolerance_deg:
                            arrived = False
                    else:
                        # Grace period: no feedback required in first 500 ms.
                        if (time.time() - start_time) < 0.5:
                            arrived = False

                # 3) Update status every 250 ms.
                now = time.time()
                if now - last_status_update > 0.25:
                    parts = []
                    for idx, _, _ in msgs:
                        t = targets_deg[idx]
                        p = positions.get(idx, float('nan'))
                        parts.append(f"J{idx + 1}: {p:.1f}/{t:.1f}")
                    elapsed = time.time() - start_time
                    self._set_status(
                        f"streaming ({frame_count} fr, {elapsed:.1f}s) | "
                        + " | ".join(parts))
                    last_status_update = now

                if arrived:
                    self._set_status(
                        f"OK \u2014 reached target "
                        f"({frame_count} frames, {time.time() - start_time:.1f}s)")
                    return

                # Pace to the control rate.
                remaining = interval - (time.time() - tick_start)
                if remaining > 0:
                    self._stop_event.wait(remaining)

            # Loop exited: timeout or stop requested.
            if self._stop_event.is_set():
                self._set_status("stopped by user")
            else:
                self._set_status(f"timeout after {frame_count} frames")

        except Exception as e:
            self._set_status(f"ERROR: {e}")
        finally:
            self._disable_all()
            self._close_bus()

