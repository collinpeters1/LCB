# modules/LightPolicy.py
# Coverage-minimizing policy using your PIDController.PIDLike.
# Goal (per column): cover the set of dark sectors with the FEWEST lights
# while avoiding lighting non-dark sectors. PID drives chosen lights; others ramp down.
#
# Hardware: LTC1665 on SPI1 (Pi 5)
# Column 1 -> CE1 (device=1), Column 2 -> CE0 (device=0)

from __future__ import annotations
import spidev
from modules.PIDController import PIDLike, PIDLikeConfig  # your PID

# --- SPI/DAC config ---
SPI_BUS = 1
COL_TO_DEVICE = {1: 1, 2: 0}  # col -> CE device
DAC_MIN, DAC_MAX = 0, 255

# Map logical light names to (column, channel)
CHANNEL_MAP = {
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

# Sectors (row,col), order of cell_darkness: [S11,S12,S21,S22,S31,S32]
def _idx(r: int, c: int) -> int: return (r-1)*2 + (c-1)
def _is_dark(cells, r, c, thr): return float(cells[_idx(r,c)]) >= float(thr)
def _err_single(measured_dark: float, threshold_dark: int) -> float:
    # positive => too dark, needs more light
    return measured_dark - float(threshold_dark)
def _pair_error(cells, a, b, thr) -> float:
    # Use the worse (higher darkness) of the pair to push up; better one will naturally sit near threshold
    da = float(cells[_idx(*a)]); db = float(cells[_idx(*b)])
    return max(_err_single(da, thr), _err_single(db, thr))

# Assist hysteresis (based on DAC of the "parent" pair light)
ASSIST_ON  = 220   # allow single assist if its covering pair is working hard
ASSIST_OFF = 180   # shed single assist when pair relaxes

# PID controllers (one per light) using your config
_controllers: dict[str, PIDLike] | None = None
def _ensure_controllers():
    global _controllers
    if _controllers is None:
        cfg = PIDLikeConfig(
            kp=0.6, ki=0.04, kd=0.0,
            deadband=3.0,
            min_step=1.0, max_step=12.0,
            down_scale=0.5,
            integral_limit=200.0,
            out_min=DAC_MIN, out_max=DAC_MAX
        )
        _controllers = {name: PIDLike(cfg) for name in CHANNEL_MAP}

def _write_dac(value: int, channel: int, column: int):
    if not (1 <= channel <= 8): raise ValueError("Channel must be 1..8")
    if not (0 <= value <= 255): raise ValueError("Value must be 0..255")
    if column not in (1, 2):    raise ValueError("Column must be 1 or 2")
    cmd = (channel << 12) | (value << 4)  # LTC1665: CCCC DDDDDDDD 0000
    hi = (cmd >> 8) & 0xFF; lo = cmd & 0xFF
    dev = COL_TO_DEVICE[column]
    spi = spidev.SpiDev(); spi.open(SPI_BUS, dev)
    spi.max_speed_hz = 2_000_000; spi.mode = 0
    spi.writebytes([hi, lo]); spi.close()

def _push_all(dac_values: dict[str,int]):
    for name, val in dac_values.items():
        col, ch = CHANNEL_MAP[name]
        _write_dac(int(val), ch, column=col)

def init_state() -> dict[str,int]:
    return {k: 0 for k in CHANNEL_MAP}

def reset_all(state: dict[str,int]) -> dict[str,int]:
    for (col, ch) in CHANNEL_MAP.values():
        _write_dac(0, ch, column=col)
    for k in state: state[k] = 0
    _ensure_controllers()
    for ctl in _controllers.values(): ctl.reset()
    return state

# ---- Coverage-minimizing planner (per column) ----
# For Column 1: sectors are (1,1)=S11, (2,1)=S21, (3,1)=S31
# Available lights: singles HS11 (S11), HS21 (S21), HF1 (S31); pairs LS1 (S11+S21), LF1 (S21+S31)
def _plan_column1(cells, thr):
    d11 = _is_dark(cells,1,1,thr); d21 = _is_dark(cells,2,1,thr); d31 = _is_dark(cells,3,1,thr)
    on = set()
    if not (d11 or d21 or d31):
        return on  # none
    if d11 and d21 and not d31:
        on |= {"LS1"}
    elif d21 and d31 and not d11:
        on |= {"LF1"}
    elif d11 and d31 and not d21:
        on |= {"HS11","HF1"}  # avoid lighting S21
    elif d11 and not d21 and not d31:
        on |= {"HS11"}
    elif d21 and not d11 and not d31:
        on |= {"HS21"}        # ← your S21-only case
    elif d31 and not d11 and not d21:
        on |= {"HF1"}
    else:  # all three dark
        on |= {"LS1","LF1"}   # covers all with 2 lights
    return on

# Column 2 mirrors Column 1 naming
def _plan_column2(cells, thr):
    d12 = _is_dark(cells,1,2,thr); d22 = _is_dark(cells,2,2,thr); d32 = _is_dark(cells,3,2,thr)
    on = set()
    if not (d12 or d22 or d32):
        return on
    if d12 and d22 and not d32:
        on |= {"LS2"}
    elif d22 and d32 and not d12:
        on |= {"LF2"}
    elif d12 and d32 and not d22:
        on |= {"HS12","HF2"}
    elif d12 and not d22 and not d32:
        on |= {"HS12"}
    elif d22 and not d12 and not d32:
        on |= {"HS22"}
    elif d32 and not d12 and not d22:
        on |= {"HF2"}
    else:  # all three dark
        on |= {"LS2","LF2"}
    return on

# Map singles to their covering pair for assist gating
PARENT_PAIR = {
    "HS11": "LS1",
    "HS21": "LS1",  # S21 sits in both pairs; we allow either parent to gate assist below
    "HF1":  "LF1",
    "HS12": "LS2",
    "HS22": "LS2",
    "HF2":  "LF2",
}
# For HS21/HS22 (middle row), also consider the *other* pair as parent
ALT_PARENT = {"HS21": "LF1", "HS22": "LF2"}

def step_and_apply(cell_darkness: list[float|int],
                   state: dict[str,int],
                   *,
                   step: int,                   # kept for compatibility (unused by PIDLike)
                   threshold_dark: int,
                   emergency_mode: bool) -> dict[str,int]:
    """
    Decide a minimal-coverage plan per column, run PID on chosen lights.
    Non-planned lights ramp down (and reset). Singles may assist their sector
    IF their covering pair is saturated (>=ASSIST_ON) and that sector remains dark.
    """
    _ensure_controllers()

    # Emergency override: zero all, ramp LF2 up
    if emergency_mode:
        for k in state: state[k] = 0
        state["LF2"] = min(state["LF2"] + 15, DAC_MAX)
        for ctl in _controllers.values(): ctl.reset()
        _push_all(state)
        return state

    # ---- Build minimal-coverage plans ----
    plan = set()
    plan |= _plan_column1(cell_darkness, threshold_dark)
    plan |= _plan_column2(cell_darkness, threshold_dark)

    # Convenience: read darkness
    S11 = float(cell_darkness[_idx(1,1)]); S12 = float(cell_darkness[_idx(1,2)])
    S21 = float(cell_darkness[_idx(2,1)]); S22 = float(cell_darkness[_idx(2,2)])
    S31 = float(cell_darkness[_idx(3,1)]); S32 = float(cell_darkness[_idx(3,2)])

    # ---- Primary PID updates (planned lights) ----
    def pid_update(name: str, error: float):
        state[name] = _controllers[name].update(error, state[name])
        if state[name] < DAC_MIN: state[name] = DAC_MIN
        if state[name] > DAC_MAX: state[name] = DAC_MAX

    # Column 1 planned
    if "LS1" in plan:
        pid_update("LS1", _pair_error(cell_darkness, (1,1),(2,1), threshold_dark))
    else:
        state["LS1"] = max(state["LS1"]-5, DAC_MIN); _controllers["LS1"].reset()
    if "LF1" in plan:
        pid_update("LF1", _pair_error(cell_darkness, (2,1),(3,1), threshold_dark))
    else:
        state["LF1"] = max(state["LF1"]-5, DAC_MIN); _controllers["LF1"].reset()

    if "HS11" in plan:
        pid_update("HS11", _err_single(S11, threshold_dark))
    else:
        # may assist if LS1 saturated and S11 still dark
        if _is_dark(cell_darkness,1,1,threshold_dark) and state["LS1"] >= ASSIST_ON:
            pid_update("HS11", _err_single(S11, threshold_dark))
        else:
            state["HS11"] = max(state["HS11"]-5, DAC_MIN); _controllers["HS11"].reset()

    if "HS21" in plan:
        pid_update("HS21", _err_single(S21, threshold_dark))
    else:
        # may assist if either LS1 or LF1 saturated and S21 still dark
        if _is_dark(cell_darkness,2,1,threshold_dark) and (
            state["LS1"] >= ASSIST_ON or state["LF1"] >= ASSIST_ON
        ):
            pid_update("HS21", _err_single(S21, threshold_dark))
        else:
            state["HS21"] = max(state["HS21"]-5, DAC_MIN); _controllers["HS21"].reset()

    if "HF1" in plan:
        pid_update("HF1", _err_single(S31, threshold_dark))
    else:
        if _is_dark(cell_darkness,3,1,threshold_dark) and state["LF1"] >= ASSIST_ON:
            pid_update("HF1", _err_single(S31, threshold_dark))
        else:
            state["HF1"] = max(state["HF1"]-5, DAC_MIN); _controllers["HF1"].reset()

    # Column 2 planned
    if "LS2" in plan:
        pid_update("LS2", _pair_error(cell_darkness, (1,2),(2,2), threshold_dark))
    else:
        state["LS2"] = max(state["LS2"]-5, DAC_MIN); _controllers["LS2"].reset()
    if "LF2" in plan:
        pid_update("LF2", _pair_error(cell_darkness, (2,2),(3,2), threshold_dark))
    else:
        state["LF2"] = max(state["LF2"]-5, DAC_MIN); _controllers["LF2"].reset()

    if "HS12" in plan:
        pid_update("HS12", _err_single(S12, threshold_dark))
    else:
        if _is_dark(cell_darkness,1,2,threshold_dark) and state["LS2"] >= ASSIST_ON:
            pid_update("HS12", _err_single(S12, threshold_dark))
        else:
            state["HS12"] = max(state["HS12"]-5, DAC_MIN); _controllers["HS12"].reset()

    if "HS22" in plan:
        pid_update("HS22", _err_single(S22, threshold_dark))
    else:
        if _is_dark(cell_darkness,2,2,threshold_dark) and (
            state["LS2"] >= ASSIST_ON or state["LF2"] >= ASSIST_ON
        ):
            pid_update("HS22", _err_single(S22, threshold_dark))
        else:
            state["HS22"] = max(state["HS22"]-5, DAC_MIN); _controllers["HS22"].reset()

    if "HF2" in plan:
        pid_update("HF2", _err_single(S32, threshold_dark))
    else:
        if _is_dark(cell_darkness,3,2,threshold_dark) and state["LF2"] >= ASSIST_ON:
            pid_update("HF2", _err_single(S32, threshold_dark))
        else:
            state["HF2"] = max(state["HF2"]-5, DAC_MIN); _controllers["HF2"].reset()

    # If a pair light relaxed far enough, shed any single assist that was on
    for single, parent in PARENT_PAIR.items():
        if state[parent] <= ASSIST_OFF:
            state[single] = max(state[single]-5, DAC_MIN)

    _push_all(state)
    return state
