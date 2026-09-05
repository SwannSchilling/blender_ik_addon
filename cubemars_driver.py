"""CubeMars AK-series actuator driver for the PickIK arm7 Blender addon.

Manages one shared CAN bus (canable gs_usb / gs-USB2 family on Windows)
with up to 7 actuators. All CAN I/O happens on background threads so the
Blender UI never blocks.

Bus model (v1.2.6+): the adapter allows exactly ONE open handle at a
time, so the driver opens the bus ONCE and keeps it alive across
Send/Stop/telemetry cycles. Only disconnect() (or a config change /
plugin unload) closes it. This matters because on Windows the gs_usb
open path performs a USB-level ResetDevice, and WinUSB does not
reliably allow a re-open right after such a reset - a fresh open on
every Send is what left the adapter unreachable after the first move.

Two upstream defects are worked around here (do not "clean up" without
re-reading this):

1. pyusb >= 1.0 (1.3.x): Device has no close()/is_opened; the libusb
   handle is owned by Device._ctx and is only released by the
   AutoFinalizedObject finalizer (i.e. when the Device object is
   garbage-collected). GC timing is not a reliable release mechanism in
   a long-lived process, so we release it explicitly via
   Device.finalize() (_close_pyusb_device).

2. gs_usb.GsUsb.start() calls Device.reset() on every open ("Reset to
   support restart multiple times" - harmless on Linux, fatal on
   Windows: after ResetDevice the next libusb_open fails with
   ACCESS_DENIED, even when every handle was closed properly). The
   gs_usb firmware is already brought to a clean state by the CAN MODE
   RESET control transfer (GsUsb.stop()), so the USB-level reset is
   skipped (_install_no_usb_reset_patch).

Prerequisites: python-can + gs_usb (the panel "Install deps" button
pip-installs them into Blender's own Python in the background).

The adapter must have the WinUSB driver installed (via Zadig).
"""

from __future__ import annotations

import importlib
import os
import struct
import subprocess
import sys
import threading
import time


def get_dep_dir() -> str:
    """Per-user, per-Python-version directory where 'Install deps' puts
    python-can + gs_usb.

    Why not a plain 'pip install': Blender's embedded Python disables the
    user site (site.ENABLE_USER_SITE is False), so a '--user' install is
    invisible to Blender even after a restart; a plain install targets
    Program Files, which a non-admin pip cannot write. '--target' into
    this writable directory plus the sys.path bootstrap below is the
    only arrangement that survives a restart.
    """
    pyv = "py%d%d" % (sys.version_info[0], sys.version_info[1])
    if sys.platform == "win32":
        root = os.environ.get("APPDATA", os.path.expanduser("~"))
        return os.path.join(root, "Blender Foundation", "Blender",
                            "python-deps", pyv)
    return os.path.join(os.path.expanduser("~"), ".local", "share",
                        "pickik-arm7", "python-deps", pyv)


_DEP_DIR = get_dep_dir()


def _ensure_dep_path() -> None:
    """Put the dep dir on sys.path if present (idempotent)."""
    if not os.path.isdir(_DEP_DIR):
        return
    _dep_norm = os.path.normcase(os.path.abspath(_DEP_DIR))
    if _dep_norm not in [os.path.normcase(os.path.abspath(p))
                         for p in sys.path]:
        sys.path.append(_DEP_DIR)


# Make a previously installed copy visible to THIS interpreter before the
# import below (and every later import) looks for 'can'.
_ensure_dep_path()

# On Windows, ensure libusb-1.0.dll (required by pyusb/gs_usb) is findable.
# It can live next to this module (copied by the 'Install deps' flow) or
# already be in the system PATH.
if sys.platform == "win32":
    _dll_dir = os.path.dirname(os.path.abspath(__file__))
    _dll_path = os.path.join(_dll_dir, "libusb-1.0.dll")
    if os.path.isfile(_dll_path) and _dll_dir not in os.environ.get("PATH", ""):
        os.environ["PATH"] = _dll_dir + os.pathsep + os.environ.get("PATH", "")
    # Also try the deps dir
    if os.path.isdir(_DEP_DIR):
        _dll2 = os.path.join(_DEP_DIR, "libusb-1.0.dll")
        if os.path.isfile(_dll2) and _DEP_DIR not in os.environ.get("PATH", ""):
            os.environ["PATH"] = _DEP_DIR + os.pathsep + os.environ.get("PATH", "")

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


def pack_set_origin() -> bytes:
    """Pack the Mode 5 (Set Origin / set zero point) 8-byte payload.

    All zeros: the firmware re-references its encoder count so that the
    motor's CURRENT position becomes 0 deg (CubeMars manual V3.2.0,
    mode 5). Not stored in the motor: the origin is lost when the
    actuator loses power.
    """
    return b"\x00" * 8


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
# DEPENDENCY PROBING + IN-APP INSTALL
# ============================================================================

DEP_PACKAGES = ("python-can", "gs_usb")


def _gs_usb_interface_ready() -> bool:
    """True if python-can's gs_usb bus class is importable in this
    interpreter (the gs_usb PyPI package provides the native bindings;
    without it the gs_29usb module itself fails to import)."""
    if can is None:
        return False
    for name in ("can.interfaces.gs_29usb", "can.interfaces.gs_usb"):
        try:
            mod = importlib.import_module(name)
        except ImportError:
            continue
        except Exception:
            # Module exists but its native dep is missing -> not ready.
            return False
        for obj in vars(mod).values():
            # Old python-can: GS_USBCanBus in can.interfaces.gs_29usb.
            # New python-can: GsUsbBus / GsUsb in can.interfaces.gs_usb.
            if (isinstance(obj, type)
                    and obj.__name__.upper().replace("_", "").startswith(
                        "GSUSB")):
                return True
    return False


def check_dependencies() -> dict:
    """Report which Python deps are importable in the running interpreter
    (Blender's). Never raises - the panel reads it every redraw.

    Returns {"can": bool, "can_version": str, "gs_usb": bool, "ready": bool}
    where "can" is the python-can package and "gs_usb" is the native
    bindings the gs_usb CAN interface needs.
    """
    info = {"can": _CAN_AVAILABLE, "can_version": "", "gs_usb": False,
            "ready": False}
    if _CAN_AVAILABLE:
        try:
            info["can_version"] = str(getattr(can, "__version__", "unknown"))
        except Exception:
            pass
        info["gs_usb"] = _gs_usb_interface_ready()
    info["ready"] = bool(info["can"] and info["gs_usb"])
    return info


def install_dependencies(timeout: int = 600) -> str:
    """pip-install python-can + gs_usb where THIS interpreter can see
    them.

    Runs pip as a subprocess of this interpreter's executable with
    '--target <dep dir>': the packages land in a writable per-user,
    per-Python-version directory (get_dep_dir) that the module already
    keeps on sys.path - so they are importable immediately (after a
    cache refresh) and after every restart. A plain 'pip install' is
    not viable here: Blender's embedded Python disables the user site
    (a '--user' install would be invisible even after a restart), and a
    system site-packages under Program Files needs admin.
    Blocks - call from a background thread.
    Returns a human-readable outcome string.
    """
    global can, _CAN_AVAILABLE
    try:
        import pip  # noqa: F401  (fail fast with a clear message)
    except ImportError:
        return ("ERROR: pip is not available in this Blender's Python "
                "(%s). Install manually: %s -m pip install --target %s "
                "python-can gs_usb" % (sys.executable, sys.executable,
                                       _DEP_DIR))
    try:
        os.makedirs(_DEP_DIR, exist_ok=True)
    except Exception as e:
        return "ERROR: cannot create dep dir %s: %s" % (_DEP_DIR, e)
    cmd = [sys.executable, "-m", "pip", "install", "--upgrade",
           "--disable-pip-version-check",
           "--target", _DEP_DIR, *DEP_PACKAGES]
    out = ""
    proc = None
    for attempt in (1, 2):  # one retry: pip flakes (cache locks, network)
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=timeout)
        except subprocess.TimeoutExpired:
            return "ERROR: pip timed out after %d s" % timeout
        except Exception as e:
            return "ERROR: pip failed to start: %s" % e
        out = (proc.stderr or proc.stdout or "").strip()
        if proc.returncode == 0:
            break
        if attempt == 1:
            print("[PickIK] pip failed (rc %s), retrying once..."
                  % proc.returncode)
    if proc.returncode != 0:
        return "ERROR: pip install failed:\n" + out[-500:]
    # Refresh the in-process module state so 'import can' works now:
    # (a) the dep dir may not have existed when this module was imported
    #     (first-ever install), so re-assert the path;
    # (b) pip --upgrade may have replaced files under already-imported
    #     modules, so drop them and re-import fresh.
    _ensure_dep_path()
    stale = ("can", "gs_usb", "wrapt", "usb", "packaging",
             "typing_extensions", "serial")
    for name in [n for n in list(sys.modules)
                 if any(n == s or n.startswith(s + ".") for s in stale)]:
        del sys.modules[name]
    importlib.invalidate_caches()
    try:
        import can as _can_mod
        can = _can_mod
        _CAN_AVAILABLE = True
    except ImportError:
        pass
    deps = check_dependencies()
    if deps["ready"]:
        return ("OK: installed python-can %s + gs_usb into %s "
                "(usable right away, and after restarts)"
                % (deps["can_version"], _DEP_DIR))
    return ("WARNING: pip finished (into %s) but the driver deps still "
            "do not import in this Blender - a restart may be needed\n%s"
            % (_DEP_DIR, out[-300:]))


# ============================================================================
# BUS EVENT LOG (diagnostics)
# ============================================================================

_bus_log_lock = threading.Lock()
_bus_log: list = []  # (ts, thread, event, detail) - last 20 entries


def _log_bus_event(event: str, detail: str = "") -> None:
    entry = (time.time(), threading.current_thread().name[:16], event,
             detail[:200])
    with _bus_log_lock:
        _bus_log.append(entry)
        del _bus_log[:-20]


def bus_event_log(limit: int = 20) -> list:
    """Copy of the recent bus lifecycle events (for diagnostics)."""
    with _bus_log_lock:
        return list(_bus_log[-limit:])


def _bus_events_brief() -> str:
    with _bus_log_lock:
        items = list(_bus_log[-6:])
    if not items:
        return "no bus events recorded yet"
    t0 = items[0][0]
    return "; ".join(
        ('%s (+%.0fs)%s' % (t[2], t[0] - t0, (' ' + t[3]) if t[3] else ''))
        for t in items)


# ============================================================================
# HANDLE RELEASE + USB-RESET SUPPRESSION
# ============================================================================

def _close_pyusb_device(dev) -> None:
    """Release the libusb/WinUSB handle of a pyusb Device across versions.

    pyusb >= 1.0: the handle lives in Device._ctx and is released by the
    AutoFinalizedObject finalizer (Device has NO close()/is_opened) -
    we trigger it explicitly via Device.finalize().
    pyusb 0.x:    Device.close() after the is_opened check.
    """
    if dev is None:
        return
    finalize = getattr(dev, "finalize", None)
    if callable(finalize):
        try:
            finalize()
            return
        except Exception:
            pass
    try:
        dev.close()
    except Exception:
        pass


_NO_USB_RESET_INSTALLED = False


def _install_no_usb_reset_patch() -> None:
    """Stop gs_usb.GsUsb.start() from issuing a USB-level ResetDevice.

    gs_usb's GsUsb.start() opens with 'self.gs_usb.reset()' ('Reset to
    support restart multiple times'). On Linux that is harmless; on
    Windows + WinUSB it is what makes the adapter unreachable: after
    ResetDevice the WinUSB kernel object does not reliably accept the
    next libusb_open (ACCESS_DENIED), even when every handle was closed
    properly. The CAN controller itself is already reset by the MODE
    RESET control transfer in GsUsb.stop(), so the USB reset adds risk
    without benefit. We no-op Device.reset() in-process; the CAN-level
    reset still happens on every close.
    """
    global _NO_USB_RESET_INSTALLED
    if _NO_USB_RESET_INSTALLED:
        return
    try:
        import usb.core as _uc
    except ImportError:
        return
    if getattr(_uc.Device, "_pickik_no_usb_reset", False):
        _NO_USB_RESET_INSTALLED = True
        return

    def _reset_noop(self):
        # Deliberately no USB reset - see the function docstring.
        return None

    _uc.Device.reset = _reset_noop
    _uc.Device._pickik_no_usb_reset = True
    _NO_USB_RESET_INSTALLED = True
    _log_bus_event("patch", "usb Device.reset no-op'd (WinUSB safety)")
    print("[PickIK] CubeMars: USB-level reset suppressed "
          "(WinUSB re-open safety)")


def _suppress_partial_bus_warnings(exc: BaseException) -> None:
    """Mark partially-constructed python-can bus objects left behind by a
    FAILED open as shut down.

    GsUsbBus.__init__ sets _is_shutdown=False before its start() fails,
    so a leaked partial bus runs GsUsbBus.shutdown() from BusABC.__del__
    at GC time - a 're-init dance' that re-scans the USB bus and, on
    Windows, can crash libusb enumeration with an access violation.
    Walking the exception chain's tracebacks finds the partial object
    (as a frame local) and silences that path."""
    chain: list[BaseException] = []
    seen_exc: set[int] = set()
    cur: BaseException | None = exc
    while cur is not None and id(cur) not in seen_exc:
        seen_exc.add(id(cur))
        chain.append(cur)
        cur = cur.__context__ or cur.__cause__
    seen_obj: set[int] = set()
    for e in chain:
        tb = e.__traceback__
        while tb is not None:
            for obj in tb.tb_frame.f_locals.values():
                try:
                    flag = getattr(obj, "_is_shutdown", None)
                except Exception:
                    continue
                if flag is False and id(obj) not in seen_obj:
                    seen_obj.add(id(obj))
                    try:
                        obj._is_shutdown = True
                    except Exception:
                        pass
            tb = tb.tb_next


def _open_thread_safe_bus(interface: str, channel: int, bitrate: int,
                          attempts: int = 3, wait: float = 0.4):
    """Open a python-can ThreadSafeBus with bounded retries for the
    WinUSB transient states (ACCESS_DENIED / device not (yet) found)
    that can briefly follow a device re-attach. Raises the last error."""
    for attempt in range(1, attempts + 1):
        try:
            return can.ThreadSafeBus(interface=interface, channel=channel,
                                     bitrate=bitrate)
        except Exception as e:
            # Every failed attempt leaves a partially-constructed bus in
            # the traceback with _is_shutdown=False; silence its __del__
            # re-init dance before the exception (and its frames) are
            # dropped - including on retried attempts.
            _suppress_partial_bus_warnings(e)
            if attempt == attempts:
                raise
            low = str(e).lower()
            if not any(k in low for k in ("access denied", "devices found",
                                          "not found", "no device")):
                raise
            _log_bus_event("open-retry",
                           "attempt %d/%d: %s" % (attempt, attempts,
                                                  str(e)[:80]))
            print("[PickIK] CubeMars: adapter not ready yet "
                  "(attempt %d/%d), retrying in %.1f s..."
                  % (attempt, attempts, wait))
            time.sleep(wait)


def _release_bus(bus) -> None:
    """Close a python-can bus AND release the underlying WinUSB handle.

    Upstream gaps (python-can 4.6.x + gs_usb + pyusb 1.x):
      * GsUsbBus.shutdown() never releases the pyusb handle (in pyusb
        >= 1.0 the handle is only freed when the Device object is
        garbage-collected - not a reliable release in a long-lived
        process);
      * its 're-init' dance opens a SECOND handle (scan -> start ->
        stop on a fresh GsUsb) and abandons it;
      * pyusb >= 1.0 Device has no close()/is_opened, so the classic
        dev.close() is a no-op there.
    Therefore: send the CAN MODE RESET control transfer ourselves
    (equivalent of gs_usb's stop), explicitly release the device
    handle, and NEVER call the interface's own shutdown() for the
    gs_usb backend. All other backends: plain shutdown().
    """
    if bus is None:
        return
    g = None
    try:
        # ThreadSafeBus is an ObjectProxy that forwards attribute access
        # to the wrapped GsUsbBus, which holds the gs_usb.GsUsb wrapper.
        g = getattr(bus, "gs_usb", None)
    except Exception:
        g = None
    if g is not None:
        try:
            g.stop()  # CAN protocol reset (what shutdown does first)
        except Exception:
            pass
        _close_pyusb_device(getattr(g, "gs_usb", None))
        try:
            # Mark the interface as shut down so BusABC.__del__ does not
            # re-run GsUsbBus.shutdown() at GC time: that would open a
            # second handle and re-scan the USB bus, which can crash with
            # a libusb access violation. We have already released the
            # handle above, so the interface is fully closed.
            wrapped = getattr(bus, "__wrapped__", bus)
            wrapped._is_shutdown = True
        except Exception:
            pass
        _log_bus_event("release", "CAN MODE RESET + pyusb handle released")
        return
    try:
        bus.shutdown()
    except Exception:
        pass


# ============================================================================
# DRIVER
# ============================================================================

N_MAX_MOTORS = 7


class CubeMarsDriver:
    """Manages one shared CAN bus with up to 7 CubeMars AK-series
    actuators.

    Bus lifecycle: the CAN bus is opened ONCE (lazily, on the first
    operation) and stays open until disconnect() is called - across
    Send/Stop cycles, telemetry reads, and driver checks. The adapter
    only allows one open handle at a time, and on Windows a re-open
    performs a disruptive USB reset; keeping one handle alive avoids
    both problems. All CAN I/O happens on background threads so the
    Blender UI never blocks. The caller (Blender operator) calls
    stream_to_targets / read_telemetry / check_driver, which are
    non-blocking; a Blender timer polls the 'status' property.
    """

    def __init__(self, interface: str = "gs_usb", channel: int = 0,
                 bitrate: int = 1_000_000,
                 motor_ids: list[int] | None = None,
                 directions: list[int] | None = None,
                 bus=None):
        """
        Args:
            interface: python-can interface name.
            channel:   Channel index.
            bitrate:   CAN bus speed.
            motor_ids: List of 7 CAN drive IDs (0 = inactive).
            directions: List of 7 per-joint direction signs (+1 = the
                       motor's + rotation matches the rig joint, -1 =
                       inverted). A motor that is physically mounted /
                       geared so that the rig's + angle drives it the
                       other way gets -1; the sign is applied to the
                       commanded position in BOTH the one-shot stream
                       and the live-update stream (and to the arrival
                       check). Per-joint invert toggles in the panel
                       are roadmap (v1.3.0) - today the addon passes a
                       hardcoded table.
            bus:       Optional pre-opened can.Bus to reuse. If provided,
                       the driver will NOT open/close the bus itself.
        """
        if motor_ids is None:
            motor_ids = [0] * N_MAX_MOTORS
        if len(motor_ids) != N_MAX_MOTORS:
            raise ValueError(f"motor_ids must have exactly {N_MAX_MOTORS} entries")
        if directions is None:
            directions = [1] * N_MAX_MOTORS
        if len(directions) != N_MAX_MOTORS:
            raise ValueError(f"directions must have exactly {N_MAX_MOTORS} entries")
        self._interface = interface
        self._channel = channel
        self._bitrate = bitrate
        self._motor_ids = list(motor_ids)
        # Normalize any sign-carrying value to a clean +1/-1.
        self._directions = [1 if d >= 0 else -1 for d in directions]
        self._active_idx = [i for i, m in enumerate(motor_ids) if m != 0]
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._status: str = "idle"
        self._bus = bus
        self._owns_bus = bus is None
        self._bus_lock = threading.Lock()
        self._bus_opened_ts: float = 0.0
        self._bus_dead: bool = False
        # Live-update mode: a continuous stream that follows the arm
        # pose; update_live_targets() moves the target it tracks.
        self._live_mode: bool = False
        self._live_lock = threading.Lock()
        self._live_targets: list | None = None
        self._live_last_update_ts: float = 0.0

    # ------------------------------------------------------------------
    # state
    # ------------------------------------------------------------------

    @property
    def is_active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    @property
    def status(self) -> str:
        with self._lock:
            return self._status

    @property
    def bus_is_open(self) -> bool:
        with self._bus_lock:
            return self._bus is not None and not self._bus_dead

    @property
    def is_live(self) -> bool:
        """True while a live-update (continuous, pose-following) stream
        is configured - the arm's new pose is streamed to the motors."""
        return self._live_mode

    def _set_status(self, text: str) -> None:
        with self._lock:
            self._status = text

    # ------------------------------------------------------------------
    # persistent bus lifecycle
    # ------------------------------------------------------------------

    def ensure_bus(self) -> None:
        """Open the CAN bus once and keep it open. Thread-safe.

        Re-opened from scratch (fresh USB scan) only after the adapter
        has been lost mid-operation (_bus_dead) or after disconnect().
        """
        with self._bus_lock:
            if self._bus is not None and not self._bus_dead:
                return
            if not self._owns_bus:
                if self._bus is None:
                    raise RuntimeError(
                        "No bus available (shared bus was None).")
                return
            # Owned bus that is (dead or) missing: open a fresh one.
            if self._bus is not None:
                stale, self._bus = self._bus, None
                _log_bus_event("drop-dead-handle", "")
                _release_bus(stale)
            _install_no_usb_reset_patch()
            try:
                bus = _open_thread_safe_bus(
                    self._interface, self._channel, self._bitrate)
            except Exception as e:
                _log_bus_event("open-fail", str(e)[:160])
                raise RuntimeError(
                    self._open_error_message(e)) from e
            self._bus = bus
            self._bus_opened_ts = time.time()
            self._bus_dead = False
            _log_bus_event("open",
                           "%s ch%d @ %d bit/s"
                           % (self._interface, self._channel,
                              self._bitrate))
            print("[PickIK] CubeMars: CAN bus opened (kept open until "
                  "Disconnect): %s ch%d @ %d bit/s"
                  % (self._interface, self._channel, self._bitrate))

    def _open_error_message(self, e: Exception) -> str:
        """Turn a bus-open failure into an actionable message."""
        msg = str(e).strip()
        low = msg.lower()
        if "access denied" in low:
            hint = (" - the adapter is still held by another process "
                    "(close other Blender/Python sessions) or by a stale "
                    "handle; unplug + replug the USB cable if it persists")
        elif any(k in low for k in ("no device", "not found", "find",
                                    "no matching", "devices found")):
            hint = (" - adapter not visible to USB enumeration: unplug + "
                    "replug the cable; check the WinUSB/Zadig driver "
                    "(device 'canable gs_usb' / GS-USB2)")
        elif "libusb" in low:
            hint = " - check the USB cable and the WinUSB/Zadig driver"
        else:
            hint = ""
        return ("Failed to open CAN bus (%s ch%d): %s%s"
                % (self._interface, self._channel, msg[:200], hint))

    def _release_persistent_handle(self) -> None:
        """Close the persistent bus and release the WinUSB handle.
        A caller-provided (bus=) bus is left to its owner."""
        with self._bus_lock:
            if self._bus is None:
                return
            if not self._owns_bus:
                return
            bus = self._bus
            self._bus = None
            self._bus_dead = False
            self._bus_opened_ts = 0.0
        _release_bus(bus)

    def _mark_bus_lost(self, why: str) -> None:
        """The adapter failed at the USB level mid-operation: drop the
        handle so the next operation re-opens from a fresh USB scan."""
        with self._bus_lock:
            self._bus_dead = True
        _log_bus_event("lost", why)
        self._release_persistent_handle()

    def disconnect(self) -> None:
        """Stop streaming (if active) and close the CAN bus, releasing
        the WinUSB handle. Call before changing interface/channel, on
        plugin unload, or to recover a stuck adapter. Send re-opens on
        demand. Blocks until the streaming thread has fully exited."""
        self.stop()
        self._release_persistent_handle()
        _log_bus_event("disconnect", "")
        print("[PickIK] CubeMars: CAN bus closed (adapter released)")

    # Backwards-compatible alias (old call sites: close() == full stop +
    # bus release).
    close = disconnect

    def stop(self) -> None:
        """Signal the streaming thread to stop, disable the motors, and
        wait for the thread to fully exit (usually under 0.6 s). The
        bus itself stays open (persistent model) - use disconnect() to
        release the adapter."""
        if self._thread is None:
            self._live_mode = False
            return
        self._stop_event.set()
        self._set_status("stopping...")
        self._thread.join(timeout=3.0)
        if not self._thread.is_alive():
            self._thread = None
        self._live_mode = False

    # ------------------------------------------------------------------
    # streaming
    # ------------------------------------------------------------------

    def stream_to_targets(self, targets_deg: list[float],
                          speed_erpm: float = 2000.0,
                          accel_erpm_s2: float = 2000.0,
                          send_hz: float = 50.0,
                          tolerance_deg: float = 2.0,
                          timeout: float = 15.0) -> None:
        """Start streaming position commands to all active motors
        (non-blocking). If a previous stream is still finishing after a
        Stop, this waits for it instead of failing the click."""
        if not _CAN_AVAILABLE:
            raise RuntimeError(
                "python-can is not installed. Run: pip install python-can gs_usb")
        if not self._active_idx:
            raise RuntimeError("No active motors configured (all IDs are 0).")
        if self.is_active:
            self._stop_event.set()
            self._thread.join(timeout=3.0)
            if self.is_active:
                raise RuntimeError(
                    "Previous stream still running - press Stop first "
                    "(if it persists, use 'Disconnect adapter')")
        self._live_mode = False  # a one-shot send replaces live tracking
        self._stop_event.clear()
        self._set_status("opening bus...")
        self._thread = threading.Thread(
            target=self._worker,
            args=(targets_deg, speed_erpm, accel_erpm_s2,
                  send_hz, tolerance_deg, timeout, False),
            daemon=True,
        )
        self._thread.start()

    def start_live_streaming(self, targets_deg: list[float],
                             speed_erpm: float = 2000.0,
                             accel_erpm_s2: float = 2000.0,
                             send_hz: float = 50.0) -> None:
        """Start the live-update stream: position frames are sent to the
        active motors at send_hz continuously - the target tracked by the
        stream is moved with update_live_targets() every time the arm
        pose changes. Runs until stop() / disconnect(). Non-blocking.

        Unlike stream_to_targets() there is no arrival/timeout exit:
        position-velocity mode needs a steady frame stream to hold and
        follow the pose.
        """
        if not _CAN_AVAILABLE:
            raise RuntimeError(
                "python-can is not installed. Run: pip install python-can gs_usb")
        if not self._active_idx:
            raise RuntimeError("No active motors configured (all IDs are 0).")
        if self.is_active:
            if self._live_mode:
                # Already live: just retarget the running stream.
                with self._live_lock:
                    self._live_targets = [float(t) for t in targets_deg]
                    self._live_last_update_ts = time.time()
                return
            self._stop_event.set()
            self._thread.join(timeout=3.0)
            if self.is_active:
                raise RuntimeError(
                    "Previous stream still running - press Stop first")
        self._live_mode = True
        with self._live_lock:
            self._live_targets = [float(t) for t in targets_deg]
            self._live_last_update_ts = time.time()
        self._stop_event.clear()
        self._set_status("live update: starting...")
        self._thread = threading.Thread(
            target=self._worker,
            args=(targets_deg, speed_erpm, accel_erpm_s2,
                  send_hz, 2.0, 0.0, True),
            daemon=True,
        )
        self._thread.start()

    def update_live_targets(self, targets_deg: list[float]) -> None:
        """Move the target the live stream is tracking (thread-safe).
        No-op unless a live stream was started. Targets are in RIG space
        (degrees) - the worker applies the per-joint direction sign when
        it packs the motor frames."""
        if not self._live_mode:
            return
        with self._live_lock:
            self._live_targets = [float(t) for t in targets_deg]
            self._live_last_update_ts = time.time()

    def _apply_directions(self, targets_deg: list[float]) -> list[float]:
        """Rig-space -> motor-space: apply the per-joint direction sign
        (a -1 joint is mounted so the motor's + rotation opposes the
        rig's + angle). All targets enter the worker in rig space; this
        is the single place the sign is applied, so one-shot, live, and
        retargeting all behave the same."""
        return [t * self._directions[i] for i, t in enumerate(targets_deg)]

    def _bus_maybe_lost(self, e: Exception, what: str) -> None:
        """If 'e' looks like a USB-level failure, mark the persistent
        bus lost so the next operation re-opens it; otherwise just
        report."""
        name = type(e).__name__
        mod = (type(e).__module__ or "")
        looks_usb = ("USBError" in name or "CanOperationError" in name
                     or mod.startswith("usb") or mod.startswith("can"))
        if looks_usb:
            self._mark_bus_lost("%s: %s" % (what, name))
            self._set_status(
                "ERROR: %s - adapter lost (%s); Send will re-open the bus"
                % (what, name))
        else:
            self._set_status("ERROR: %s" % what)

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
                tolerance_deg: float, timeout: float,
                live: bool = False) -> None:
        """Background thread: stream Mode 6 frames.

        live=False (one-shot Send): runs until every active motor reports
        its target within tolerance, or until timeout / stop.
        live=True  (live update):   runs until stop(), continuously
        re-sending the current live target (update_live_targets) - there
        is no arrival/timeout exit, because position-velocity mode needs
        a steady frame stream to hold and follow the pose.
        """
        try:
            self.ensure_bus()
        except RuntimeError as e:
            self._set_status("ERROR: %s" % e)
            return

        interval = 1.0 / send_hz
        start_time = time.time()
        deadline = None if live else start_time + timeout

        can_ids = {
            idx: make_can_id(MODE_POSITION_VELOCITY, self._motor_ids[idx])
            for idx in self._active_idx
        }

        # One-shot mode pre-builds the frames once (targets are fixed);
        # live mode rebuilds each tick from the live-target holder.
        # targets_deg is rig space; eff_targets is what the motors see.
        eff_targets = self._apply_directions(targets_deg)
        msgs: list[tuple[int, object]] = []
        if not live:
            for idx in self._active_idx:
                target = eff_targets[idx]
                payload = pack_position_velocity(target, speed_erpm,
                                                 accel_erpm_s2)
                msg = can.Message(
                    arbitration_id=can_ids[idx],
                    data=list(payload),
                    is_extended_id=True,
                    dlc=len(payload),
                )
                msgs.append((idx, msg))

        fb_ids: dict[int, int] = {
            idx: make_can_id(FEEDBACK_STATUS, self._motor_ids[idx])
            for idx in self._active_idx
        }

        frame_count = 0
        last_status_update = 0.0
        fb_last_seen = 0.0  # live mode: feedback freshness (0 = never seen)

        try:
            while not self._stop_event.is_set() and (
                    deadline is None or time.time() < deadline):
                tick_start = time.time()

                if live:
                    # ---- live update: send current target every tick ----
                    with self._live_lock:
                        cur = (list(self._live_targets)
                               if self._live_targets is not None
                               else list(targets_deg))
                    eff = self._apply_directions(cur)
                    for idx in self._active_idx:
                        payload = pack_position_velocity(eff[idx], speed_erpm,
                                                         accel_erpm_s2)
                        m = can.Message(
                            arbitration_id=can_ids[idx],
                            data=list(payload),
                            is_extended_id=True,
                            dlc=len(payload),
                        )
                        try:
                            self._bus.send(m)
                        except Exception as e:
                            self._bus_maybe_lost(e, "CAN send error")
                            return
                    frame_count += 1

                    # Track feedback freshness: the motors report
                    # continuously, so silence means fault / disconnect
                    # / no power - not just "idle". Makes the common
                    # silent-failure mode visible on the panel.
                    fb_deadline = time.time() + interval * 0.5
                    while time.time() < fb_deadline:
                        try:
                            msg_in = self._bus.recv(timeout=0.005)
                        except Exception:
                            break
                        if msg_in is None:
                            continue
                        for idx, fb_id in fb_ids.items():
                            if msg_in.arbitration_id == fb_id:
                                fb_last_seen = time.time()

                    now = time.time()
                    if now - last_status_update > 0.5:
                        parts = " | ".join(
                            f"J{i + 1}: {cur[i]:.1f}" for i in self._active_idx)
                        with self._live_lock:
                            tgt_age = now - self._live_last_update_ts
                        fb_age = now - fb_last_seen
                        fb_tag = ("NO FEEDBACK (check motor power / wiring)"
                                  if fb_age > 1.0 else "fb %.1fs" % fb_age)
                        self._set_status(
                            f"live streaming ({frame_count} fr, "
                            f"{now - start_time:.1f}s) | " + parts
                            + f" | tgt {tgt_age:.1f}s old | {fb_tag}")
                        last_status_update = now

                    remaining = interval - (time.time() - tick_start)
                    if remaining > 0:
                        self._stop_event.wait(remaining)
                    continue

                # ---- one-shot Send: send pre-built frames ----
                # 1) Send one position-velocity frame to each active motor.
                for _idx, msg in msgs:
                    try:
                        self._bus.send(msg)
                    except Exception as e:
                        self._bus_maybe_lost(e, "CAN send error")
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

                # Check arrival for all active motors. positions[] is the
                # motor's OWN coordinate space (its encoder), so compare
                # against the motor-space target (eff_targets), NOT the
                # rig-space target - a -1 joint would otherwise never
                # converge and the one-shot Send would time out.
                for idx, _ in msgs:
                    if idx in positions:
                        if abs(positions[idx] - eff_targets[idx]) > tolerance_deg:
                            arrived = False
                    else:
                        # Grace period: no feedback required in first 500 ms.
                        if (time.time() - start_time) < 0.5:
                            arrived = False

                # 3) Update status every 250 ms.
                now = time.time()
                if now - last_status_update > 0.25:
                    parts = []
                    for idx, _ in msgs:
                        t = eff_targets[idx]  # motor space, like p
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

            # Loop exited: stop requested (live: no timeout path).
            if self._stop_event.is_set():
                self._set_status("stopped by user" if not live
                                 else "live update stopped")
            else:
                self._set_status(f"timeout after {frame_count} frames")

        except Exception as e:
            self._set_status(f"ERROR: {e}")
        finally:
            # NOTE: the bus is NOT closed here - it is persistent and
            # stays open for the next Send / telemetry / check. Only
            # disconnect() (or a USB-level loss) closes it. Clearing
            # the live flag here covers a stream that dies on its own
            # (e.g. bus open failed) without an explicit stop().
            self._live_mode = False
            self._disable_all()

    # ------------------------------------------------------------------
    # diagnostics
    # ------------------------------------------------------------------

    def check_driver(self) -> tuple[bool, str, str]:
        """Verify the OS driver + USB connection.

        If the persistent bus is already open, the adapter is reachable
        by definition - answering from the open bus avoids opening a
        second handle (which WinUSB would refuse). Otherwise opens a
        temporary bus once, verifies, and releases the handle properly.
        No motor commands are sent.
        Blocks for a few seconds - call from a background thread.
        Returns (ok, short_message, detail_lines).
        """
        if self.bus_is_open:
            with self._bus_lock:
                age = time.time() - self._bus_opened_ts
            return True, ("OK: %s ch%d already open (persistent bus, %.0f s)"
                          % (self._interface, self._channel, age)),                 "No new handle was opened. The bus stays open until "                 "'Disconnect adapter'."
        if self.is_active:
            return False, ("FAIL: streaming is in progress - stop it first "
                           "(the adapter allows one open handle at a time)"),                 ""
        deps = check_dependencies()
        if not deps["can"]:
            return False, ("FAIL: python-can is not installed in this "
                           "Blender"),                 "Click 'Install deps' (installs into %s)" % _DEP_DIR
        if not deps["gs_usb"]:
            return False, ("FAIL: the gs_usb bindings are missing"),                 ("Click 'Install deps' (or: %s -m pip install --target %s "
                 "gs_usb)" % (sys.executable, _DEP_DIR))
        t0 = time.time()
        _install_no_usb_reset_patch()
        try:
            bus = _open_thread_safe_bus(self._interface, self._channel,
                                        self._bitrate)
        except Exception as e:
            _log_bus_event("check-fail", str(e)[:160])
            msg = str(e).strip()
            low = msg.lower()
            if "access denied" in low:
                short = "FAIL: adapter busy (Access denied)"
                detail = ("Another process may hold the adapter, or a stale "
                          "handle is still open.\nFull error: %s\n"
                          "Hint: close other Blender/Python windows; if it "
                          "persists, unplug + replug the USB cable.\n"
                          "Recent bus events: %s") % (msg[:300],
                                                      _bus_events_brief())
            elif any(k in low for k in ("no device", "not found", "find",
                                        "no matching", "devices found")):
                short = "FAIL: adapter not found"
                detail = ("The gs_usb adapter is not visible to USB "
                          "enumeration.\nFull error: %s\n"
                          "Hint: unplug + replug the cable; check the "
                          "WinUSB/Zadig driver (device 'canable gs_usb' / "
                          "GS-USB2).\nRecent bus events: %s") % (msg[:300],
                                                                _bus_events_brief())
            else:
                short = "FAIL: bus open failed"
                detail = "Full error: %s\nRecent bus events: %s" % (
                    msg[:300], _bus_events_brief())
            return False, short, detail
        _release_bus(bus)
        return True, ("OK: %s ch%d opened in %.1f s - adapter reachable, "
                      "driver works"
                      % (self._interface, self._channel, time.time() - t0)),             "This check opened and closed the adapter once (handle "             "released)."

    def read_telemetry(self, seconds: float = 2.0) -> dict:
        """Listen to the CAN bus for 'seconds' and decode the motors'
        periodic feedback (position / speed / current / temp / error).

        Uses the persistent bus (opens it if needed). Never sends motor
        commands. Never raises - returns
        {"ok": bool, "text": str (one-line summary), "lines": [str]
        (panel detail), "frames": int, "per_motor": {idx: fb|None}}.
        """
        out: dict = {"ok": False, "text": "", "lines": [], "frames": 0,
                     "per_motor": {}}
        if not _CAN_AVAILABLE:
            out["text"] = ("Telemetry: python-can missing - click "
                           "'Install deps'")
            return out
        if not self._active_idx:
            out["text"] = "Telemetry: no motor IDs configured (all are 0)"
            return out
        if self.is_active and not self.is_live:
            # A one-shot burst owns the bus for arrival checking; a live
            # stream only SENDS frames, so telemetry may receive on the
            # same (thread-safe) bus while it runs - that is exactly the
            # moment the user needs to see where the motors actually are.
            out["text"] = ("Telemetry: streaming is in progress - "
                           "press Stop first")
            return out
        try:
            self.ensure_bus()
        except RuntimeError as e:
            out["text"] = "Telemetry: %s" % e
            out["lines"] = [str(e)]
            return out

        fb_ids = {idx: make_can_id(FEEDBACK_STATUS, self._motor_ids[idx])
                  for idx in self._active_idx}
        latest: dict = {}
        frames = 0
        ids_seen: dict = {}
        deadline = time.time() + seconds
        try:
            while time.time() < deadline:
                msg = self._bus.recv(timeout=0.05)
                if msg is None:
                    continue
                frames += 1
                key = (msg.arbitration_id, msg.is_extended_id)
                ids_seen[key] = ids_seen.get(key, 0) + 1
                idx = next((i for i, fid in fb_ids.items()
                            if fid == msg.arbitration_id), None)
                if idx is not None and len(msg.data) >= 8:
                    fb = decode_feedback(list(msg.data))
                    if fb:
                        fb["id"] = self._motor_ids[idx]
                        latest[idx] = fb
        except Exception as e:
            self._bus_maybe_lost(e, "telemetry I/O error")
            out["text"] = "Telemetry: I/O error (%s) - adapter lost?" % (
                type(e).__name__)
            out["lines"] = ["Raw frames before error: %d" % frames]
            return out

        lines: list = []
        if frames == 0:
            lines.append("Bus is open, but 0 frames received in %.1f s."
                         % seconds)
            lines.append("Motors disconnected? With the actuators unplugged "
                         "nothing is on the bus - this is expected.")
        else:
            for idx in self._active_idx:
                mid = self._motor_ids[idx]
                fb = latest.get(idx)
                if fb:
                    err = ("  [ERR: %s]" % error_name(fb["error"])
                           if fb["error"] else "")
                    lines.append(
                        "J%d (ID 0x%02X): pos %7.1f deg | speed %6.0f rpm | "
                        "current %5.2f A | temp %3d C%s"
                        % (idx + 1, mid, fb["position"], fb["speed"],
                           fb["current"], fb["temperature"], err))
                    out["per_motor"][idx] = fb
                else:
                    lines.append("J%d (ID 0x%02X): no feedback frame "
                                 "(ID wrong? powered? CAN_H/L swapped?)"
                                 % (idx + 1, mid))
            other = {k: v for k, v in ids_seen.items()
                     if k[0] not in set(fb_ids.values())}
            if other:
                lines.append("Other frames: " + ", ".join(
                    "0x%X(ext=%s) x%d" % (k[0], "Y" if k[1] else "N", v)
                    for k, v in sorted(other.items())[:6]))
        out["ok"] = True
        out["frames"] = frames
        if latest:
            out["text"] = "Telemetry: " + "; ".join(
                "J%d %s deg" % (i + 1, "%.1f" % f["position"])
                for i, f in sorted(latest.items())) + "  (%d frames)" % frames
        else:
            out["text"] = ("Telemetry: no motor feedback  (%d frames, %.1f s)"
                           % (frames, seconds))
        out["lines"] = lines
        _log_bus_event("telemetry",
                       "%d frames, %d motor(s) reporting"
                       % (frames, len(latest)))
        return out

    # ------------------------------------------------------------------
    # zero position (Set Origin)
    # ------------------------------------------------------------------

    def _sample_positions(self, seconds: float) -> dict:
        """Listen for 'seconds' and return the LAST position reported by
        each active motor ({idx: pos_deg}) - a short read-only probe.
        Motors that send no feedback frame in the window are simply
        absent from the result. Never raises."""
        latest: dict = {}
        if self._bus is None:
            return latest
        fb_ids = {idx: make_can_id(FEEDBACK_STATUS, self._motor_ids[idx])
                  for idx in self._active_idx}
        deadline = time.time() + seconds
        try:
            while time.time() < deadline:
                msg = self._bus.recv(timeout=0.05)
                if msg is None:
                    continue
                idx = next((i for i, fid in fb_ids.items()
                            if fid == msg.arbitration_id), None)
                if idx is not None and len(msg.data) >= 8:
                    fb = decode_feedback(list(msg.data))
                    if fb:
                        latest[idx] = fb["position"]
        except Exception as e:
            self._bus_maybe_lost(e, "origin-sample I/O error")
        return latest

    def set_origin(self, settle_s: float = 0.7) -> dict:
        """Set each active motor's CURRENT position as its origin (0 deg).

        Sends the CubeMars mode-5 "Set Origin" frame (CAN ID
        (5 << 8) | motor_id, 8 zero bytes) to every active motor. From
        that moment on, all position commands and the feedback position
        are referenced to the physical pose the arm was in when the
        frame was sent - exactly what the encoder-less AK motors need:
        they only count encoder steps since power-on, so "move the arm
        by hand to the physical zero pose, then set the origin" defines
        0 deg in firmware. The command itself never moves the arm - it
        only re-references the encoder count.

        The origin is NOT stored in the actuator: it is lost when the
        motor loses power (the power-on position becomes the reference
        again), so this has to be re-run after every power-up.

        Refused while any stream (one-shot or live) is running: a
        re-reference mid-stream would make the arm move.

        Never raises - returns {"ok": bool, "text": str (one-line
        summary), "lines": [str] (panel detail), "before": {idx:
        pos|None}, "after": {idx: pos|None}}. Blocks a few seconds
        (one feedback sample before and after the command) - call from
        a background thread.
        """
        out: dict = {"ok": False, "text": "", "lines": [],
                     "before": {}, "after": {}}
        if not _CAN_AVAILABLE:
            out["text"] = ("Set origin: python-can missing - click "
                           "'Install deps'")
            return out
        if not self._active_idx:
            out["text"] = ("Set origin: no motor IDs configured "
                           "(all are 0)")
            return out
        if self.is_active:
            out["text"] = ("Set origin: a stream is running (one-shot or "
                           "live) - press Stop first, then set the "
                           "origin from a still pose")
            return out
        try:
            self.ensure_bus()
        except RuntimeError as e:
            out["text"] = "Set origin: %s" % e
            out["lines"] = [str(e)]
            return out

        # 1) Sample where the motors report themselves BEFORE the command
        #    (so the readout can show "was -12.4 deg -> now 0.0 deg").
        out["before"] = self._sample_positions(settle_s)

        # 2) Send the Set Origin frame to every active motor.
        for idx in self._active_idx:
            can_id = make_can_id(MODE_SET_ORIGIN, self._motor_ids[idx])
            msg = can.Message(
                arbitration_id=can_id,
                data=list(pack_set_origin()),
                is_extended_id=True,
                dlc=8,
            )
            try:
                self._bus.send(msg)
            except Exception as e:
                self._bus_maybe_lost(e, "set-origin send")
                out["text"] = ("Set origin: I/O error (%s) - adapter "
                               "lost?" % type(e).__name__)
                out["lines"] = [str(e)]
                return out

        # 3) Sample again: the firmware re-references within one feedback
        #    period (~20 ms), so a still pose now reads ~0 deg.
        out["after"] = self._sample_positions(settle_s)

        lines: list = []
        reported = 0
        confirmed = 0
        for idx in self._active_idx:
            mid = self._motor_ids[idx]
            b = out["before"].get(idx)
            a = out["after"].get(idx)
            b_s = ("%7.1f deg" % b) if b is not None else "   (none)"
            if a is None:
                lines.append(
                    "J%d (ID 0x%02X): before %s - no feedback after the "
                    "command (motor silent? power / CAN_H-CAN_L swapped?)"
                    % (idx + 1, mid, b_s))
            elif abs(a) <= 0.5:
                reported += 1
                confirmed += 1
                lines.append(
                    "J%d (ID 0x%02X): before %s -> now %7.1f deg  "
                    "[origin set confirmed]"
                    % (idx + 1, mid, b_s, a))
            else:
                reported += 1
                lines.append(
                    "J%d (ID 0x%02X): before %s -> now %7.1f deg  "
                    "[did NOT re-reference - motor faulted or ignored "
                    "the command?]"
                    % (idx + 1, mid, b_s, a))
        if reported == 0:
            out["text"] = ("Set origin sent, but 0 motors are reporting - "
                           "check motor power / wiring, then try again")
        elif confirmed == reported:
            out["text"] = ("Set origin OK: " + "; ".join(
                "J%d -> 0.0 deg" % (i + 1) for i in self._active_idx)
                + " (this pose is now the 0 deg reference)")
        else:
            out["text"] = ("Set origin sent, but not all motors "
                           "re-referenced - see detail below")
        out["lines"] = lines
        out["ok"] = confirmed > 0 and confirmed == reported
        _log_bus_event("set-origin", "%d/%d motor(s) confirmed"
                       % (confirmed, len(self._active_idx)))
        return out
