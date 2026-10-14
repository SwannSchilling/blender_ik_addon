# SPDX-License-Identifier: BSD-3-Clause
"""Control-domain stub for the standalone SPM rig: signs, offsets and the operator-angle framing.

    import spm_driver as D                # from blender_ik_addon/ on sys.path; no Blender needed

Why this file exists separately from spm_kinematics.py. The kinematics is orientation in, angles out;
it must be sign-free and offset-free so it is the same math the HTML tool runs and can be pinned to the
golden fixture without an operator's calibration leaking in. Everything a specific bench adds - the motor
signs, the gear ratio, the per-leg hardware offsets - lives here instead. Slice 2 only needs the stub to
be the right boundary; the Blender driver of later slices fills it out.

The one asymmetry the fixtures made permanent and that this file is the guardian of: the tilt/pan/roll
shown back to the operator follows the IK construction's convention, NEVER the FK readout's (which is
display_only_from_fk_readout on the fixture and reported 60/0/0 back as pan -90). The construction that
turns tilt/pan/roll into an orientation is geometry, not calibration, so it now lives in spm_kinematics
as orientation_from_tilt_pan_roll; this module keeps ONLY the control-domain values - the motor signs,
the gear ratio, the per-leg offsets - and the display conversions, and it never claims to invert one.

Packaging. Like spm_kinematics.py this is a RUNTIME module, pure Python and bpy-free, so it belongs on
the package's explicit module list when the SPM rig ships - held off by later slices filling out the
Blender driver, not by being dev tooling.
"""
from __future__ import annotations

import math

import spm_kinematics as _K

PI = math.pi

# ---- control-domain calibration, the tool's CALIB fields that are NOT geometry --------------------
# Kept out of spm_kinematics.py on purpose: they are how this bench's motors and gears are wired, not
# how its legs are arranged. A different bench changes these and none of the kinematics.
MOTOR_SIGNS = (-1, 1, 1)
GEAR_RATIO = -0.4
# Per-leg hardware offset in degrees, added to a commanded angle before it reaches the mechanism. Zero
# until the bench is dialled in; the kinematics must never see them, so the driver applies them here.
CONTROL_OFFSET_DEG = (0.0, 0.0, 0.0)


def _deg_to_rad(d):
    return d * PI / 180.0


def solve_command(tilt_deg, pan_deg, roll_deg, mech=None, prev_R=None):
    """Command a pose and get theta[3]/valid[3]/sigma_min - the driver-facing entry the Blender panel
    will call. It builds the orientation through the kinematics' own IK construction (the tool's
    spmPoseR, now in spm_kinematics), then delegates; the driver adds no geometry of its own. Pass the
    previous tick's orientation as prev_R so the FK continues in the same assembly mode."""
    orientation = _K.orientation_from_tilt_pan_roll(tilt_deg, pan_deg, roll_deg)
    return _K.solve_from(orientation, mech)


def fk_command(theta, mech=None, prev_R=None):
    """The forward half the panel drives each tick: joint angles back to an orientation, seeded from the
    previous frame's orientation so a live solve does not jump to a mirror assembly mode. The driver is
    the thing that remembers prev_R between ticks; the kinematics stays stateless."""
    return _K.fk_to_orientation(theta, mech, prev_R)


def theta_to_display_deg(theta):
    """Joint angles to the degrees the UI prints: divide by the motor sign, as theta_display_deg does
    on the fixture. A leg with no angle is None, never a fabricated number."""
    out = []
    for i in range(3):
        t = theta[i]
        out.append(None if not math.isfinite(t) else t * 180.0 / PI / MOTOR_SIGNS[i])
    return out


def theta_from_display_deg(display_deg):
    """The inverse print: degrees shown back to joint angles, undoing the motor sign. None stays None."""
    out = []
    for i in range(3):
        d = display_deg[i]
        out.append(float("nan") if d is None else d * PI / 180.0 * MOTOR_SIGNS[i])
    return out


def apply_control_offset(theta):
    """Where a bench's per-leg hardware offset enters, if a later slice wants it in the joint domain
    rather than the operator domain. Kept here, out of the kinematics, either way."""
    return [theta[i] + _deg_to_rad(CONTROL_OFFSET_DEG[i]) if math.isfinite(theta[i]) else theta[i]
            for i in range(3)]
