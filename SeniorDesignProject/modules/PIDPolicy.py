# modules/PIDPolicy.py
# PID lighting control using simple-pid 2.0.0, with FLOOD shaping only.
#
# Behavior:
# - Each light tries to hold its covered sector(s) near threshold_dark % darkness.
# - simple_pid.PID per light.
# - Slew limiting, deadband, anti-windup.
# - Emergency mode: everything off except LF2 forced to 255.
# - Channel map can load from /home/pi/hardware_map.json if present.
#
# NEW:
# - Before sending to the DAC, flood-class lights (HF1, HF2, LF1, LF2)
#   get their logical 0..255 value remapped into a tight DAC window
#   [FLOOD_MIN_DAC..FLOOD_MAX_DAC] so that the tiny physical usable
#   range (where they actually light up) is stretched across the
#   whole control range.
#
#   Spots / other lights are passed through unchanged.

import time
import json
import spidev
from simple_pid import PID  # pip install simple-pid
from modules.FloodMap import map_state  # <-- NEW

from pathlib import Path
MAP_PATH = str(Path.home() / "hardware_map.json")


# -------------------- SPI / DAC --------------------
SPI_BUS = 1
# Column 1 -> CE1 (device=1), Column 2 -> CE0 (device=0). Flip if wired opposite.
COL_TO_DEVICE = {1: 1, 2: 0}

def _write_dac(value: int, channel: int, column: int):
    """Write 8-bit value (0..255) to LTC1665 channel on a column's SPI device."""
    v = max(0, min(255, int(value)))
    cmd = (channel << 12) | (v << 4)  # CCCC DDDDDDDD 0000
    hi = (cmd >> 8) & 0xFF
    lo = cmd & 0xFF
    dev = COL_TO_DEVICE[column]
    s = spidev.SpiDev()
    s.open(SPI_BUS, dev)
    s.max_speed_hz = 2_000_000
    s.mode = 0
    s.writebytes([hi, lo])
    s.close()

def _push_all(state: dict, channel_map: dict):
    """
    Push current state to hardware.
    We first apply per-light shaping (flood stretching).
    We do NOT mutate the caller's dict, so PID logic still "thinks"
    in normal 0..255 space.
    """
    shaped = map_state(state)  # <-- NEW
    for name, val in shaped.items():
        if name in channel_map:
            col, ch = channel_map[name]
            _write_dac(val, ch, col)

# -------------------- Channel mapping --------------------
def _load_user_map(path=MAP_PATH):
    try:
        with open(path, "r") as f:
            raw = json.load(f)  # {"HS11":[1,1], ...}
        m = {}
        for name, pair in raw.items():
            col, ch = int(pair[0]), int(pair[1])
            if col in (1, 2) and 1 <= ch <= 8:
                m[name] = (col, ch)
        if m:
            print(f"[PIDPolicy] Loaded CHANNEL_MAP from {path}")
            return m
    except Exception as e:
        print(f"[PIDPolicy] No user map loaded: {e}")
    return None

DEFAULT_CHANNEL_MAP = {
    # Column 1 (left)
    "HS11": (1, 1),
    "HS21": (1, 2),
    "LS1":  (1, 3),
    "HF1":  (1, 4),
    "LF1":  (1, 5),
    # Column 2 (right)
    "HS12": (2, 1),
    "HS22": (2, 2),
    "LS2":  (2, 3),
    "HF2":  (2, 4),
    "LF2":  (2, 5),
}

CHANNEL_MAP = _load_user_map() or DEFAULT_CHANNEL_MAP

# -------------------- Coverage map --------------------
# Sector indices:
# 0:S11  1:S12
# 2:S21  3:S22
# 4:S31  5:S32
LIGHT_TO_SECTORS = {
    "HS11": [0],
    "HS21": [2],
    "HF1":  [4],
    "LS1":  [0, 2],  # both must be dark
    "LF1":  [2, 4],  # both must be dark

    "HS12": [1],
    "HS22": [3],
    "HF2":  [5],
    "LS2":  [1, 3],  # both must be dark
    "LF2":  [3, 5],  # both must be dark
}

def _pair_active(vals, ia, ib, thresh):
    return (vals[ia] >= thresh) and (vals[ib] >= thresh)

def init_state() -> dict:
    return {k: 0 for k in CHANNEL_MAP.keys()}

# -------------------- PID Manager --------------------
class PIDManager:
    """
    One simple_pid.PID per light.
    setpoint = target darkness (threshold_dark).
    We flip sign using error_map so "too dark" => positive output.
    We also slew-limit so lights don't jump frame-to-frame.
    """
    def __init__(self,
                 kp=7.5, ki=1.2, kd=0.8,
                 output_limits=(0, 255),
                 sample_time=None,
                 slew_per_step=12.0,
                 deadband=0.5,
                 proportional_on_measurement=False,
                 differential_on_measurement=False):
        self.pids = {}
        self.slew = float(slew_per_step)
        self.deadband = float(deadband)
        self.output_limits = output_limits

        for name in CHANNEL_MAP.keys():
            pid = PID(
                kp, ki, kd,
                setpoint=0.0,
                sample_time=sample_time,
                output_limits=output_limits,
                auto_mode=True,
                proportional_on_measurement=proportional_on_measurement,
                differential_on_measurement=differential_on_measurement,
                error_map=lambda e: -e  # makes error = meas-setpoint
            )
            self.pids[name] = pid

        self.prev_out = {name: 0.0 for name in CHANNEL_MAP.keys()}

    def reset_all(self, keep_outputs=False):
        for name, pid in self.pids.items():
            last = self.prev_out.get(name, 0.0) if keep_outputs else 0.0
            pid.set_auto_mode(True, last_output=last)
            pid.auto_mode = True
            pid.integral = 0.0
            self.prev_out[name] = last

    def step(self, name: str, setpoint: float, measurement: float) -> int:
        pid = self.pids[name]
        pid.setpoint = float(setpoint)

        # deadband: if we're basically at target, lie and say we're perfect
        if abs(measurement - setpoint) < self.deadband:
            measurement = setpoint

        # for anti-windup
        err = measurement - setpoint

        out = pid(measurement)

        # slew limit
        prev = self.prev_out.get(name, 0.0)
        delta = out - prev
        if delta > self.slew:
            out = prev + self.slew
        elif delta < -self.slew:
            out = prev - self.slew

        # clamp
        lo, hi = self.output_limits
        if lo is not None: out = max(lo, out)
        if hi is not None: out = min(hi, out)

        # bleed integral if pinned
        try:
            if (out >= hi and err > 0) or (out <= lo and err < 0):
                pid.integral *= 0.5
        except Exception:
            pass

        self.prev_out[name] = out
        return int(round(out))

# -------------------- Public API --------------------
def reset_all(state: dict, manager: PIDManager) -> dict:
    for k in state.keys():
        state[k] = 0
    _push_all(state, CHANNEL_MAP)  # will map floods too
    manager.reset_all(keep_outputs=False)
    return state

def pid_step_and_apply(cell_darkness: list, state: dict, manager: PIDManager, *,
                       threshold_dark: float, emergency_mode: bool) -> dict:
    """
    cell_darkness: [S11,S12,S21,S22,S31,S32] in % dark
    threshold_dark: desired darkness (e.g., 10)
    emergency_mode: True -> all off except LF2=255 (still mapped before DAC)
    """
    if emergency_mode:
        for k in state.keys():
            state[k] = 0
        state["LF2"] = 255
        _push_all(state, CHANNEL_MAP)
        return state

    target = float(threshold_dark)

    meas = {
        "HS11": float(cell_darkness[0]),
        "HS21": float(cell_darkness[2]),
        "HF1":  float(cell_darkness[4]),
        "HS12": float(cell_darkness[1]),
        "HS22": float(cell_darkness[3]),
        "HF2":  float(cell_darkness[5]),
    }

    meas["LS1"] = min(float(cell_darkness[0]), float(cell_darkness[2])) if _pair_active(cell_darkness, 0, 2, target) else 0
    meas["LF1"] = min(float(cell_darkness[2]), float(cell_darkness[4])) if _pair_active(cell_darkness, 2, 4, target) else 0
    meas["LS2"] = min(float(cell_darkness[1]), float(cell_darkness[3])) if _pair_active(cell_darkness, 1, 3, target) else 0
    meas["LF2"] = min(float(cell_darkness[3]), float(cell_darkness[5])) if _pair_active(cell_darkness, 3, 5, target) else 0

    for light, m in meas.items():
        state[light] = manager.step(light, target, m)

    _push_all(state, CHANNEL_MAP)
    return state
