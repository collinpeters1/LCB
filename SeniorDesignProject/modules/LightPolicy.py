# modules/LightPolicy.py
# Encapsulates light ramping policy + DAC writes for both columns (LTC1665 on SPI1 CE0/CE1).
# Behavior matches your current main.py exactly.

import spidev

# --- SPI/DAC config (mirrors your main.py) ---
SPI_BUS = 1
# Column 1 -> CE1 (device=1), Column 2 -> CE0 (device=0)
COL_TO_DEVICE = {1: 1, 2: 0}

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

def _write_dac(value: int, channel: int, column: int):
    """
    Send an 8-bit value (0..255) to LTC1665 channel (1..8) on the given column (1 or 2).
    Column 1 is on SPI1 CE1, Column 2 is on SPI1 CE0.
    """
    if not (1 <= channel <= 8): raise ValueError("Channel must be 1..8")
    if not (0 <= value <= 255): raise ValueError("Value must be 0..255")
    if column not in (1, 2):    raise ValueError("Column must be 1 or 2")

    cmd = (channel << 12) | (value << 4)  # LTC1665: CCCC DDDDDDDD 0000
    hi = (cmd >> 8) & 0xFF
    lo = cmd & 0xFF

    dev = COL_TO_DEVICE[column]
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, dev)
    spi.max_speed_hz = 2_000_000
    spi.mode = 0
    spi.writebytes([hi, lo])
    spi.close()

def _push_all(state: dict):
    """Write every channel from the given state dict to the DACs."""
    for name, val in state.items():
        col, ch = CHANNEL_MAP[name]
        _write_dac(int(val), ch, column=col)

def _zero_all_dacs():
    """Force all mapped channels on both columns to 0 (hardware writes)."""
    for (col, ch) in CHANNEL_MAP.values():
        _write_dac(0, ch, column=col)

def init_state() -> dict:
    """Return a new DAC state dict with all values = 0."""
    return {k: 0 for k in CHANNEL_MAP.keys()}

def reset_all(state: dict) -> dict:
    """
    Zero hardware and return a zeroed copy of the state.
    Intended for MANUAL entry or any time you need a clean slate.
    """
    _zero_all_dacs()
    for k in state.keys():
        state[k] = 0
    return state

def step_and_apply(cell_darkness: list, state: dict, *, step: int, threshold_dark: int, emergency_mode: bool) -> dict:
    """
    AUTO mode policy. Mirrors your main.py logic, then pushes the DACs.
    - cell_darkness: list of 6 ints (S11,S12,S21,S22,S31,S32) darkness percentages
    - state: dict of DAC values (HS11, HS21, LS1, HF1, LF1, HS12, HS22, LS2, HF2, LF2)
    - step: ramp step size (e.g., 3)
    - threshold_dark: darkness threshold (e.g., 10)
    - emergency_mode: if True, zero everything except LF2 ramp to max (+15)
    Returns updated state (same dict instance).
    """

    # Index mapping for 3x2 grid in row-major:
    # 0:S11  1:S12
    # 2:S21  3:S22
    # 4:S31  5:S32
    dark_S11 = cell_darkness[0]; dark_S12 = cell_darkness[1]
    dark_S21 = cell_darkness[2]; dark_S22 = cell_darkness[3]
    dark_S31 = cell_darkness[4]; dark_S32 = cell_darkness[5]

    # Booleans for each sector exceeding threshold
    s11_dark = dark_S11 >= threshold_dark
    s21_dark = dark_S21 >= threshold_dark
    s31_dark = dark_S31 >= threshold_dark
    s12_dark = dark_S12 >= threshold_dark
    s22_dark = dark_S22 >= threshold_dark
    s32_dark = dark_S32 >= threshold_dark

    # Coupled lights per your definitions:
    # Column 1
    LS1_active = s11_dark and s21_dark       # Low spot covers S11+S21
    LF1_active = s21_dark and s31_dark       # Low flood covers S21+S31
    # Column 2
    LS2_active = s12_dark and s22_dark       # Low spot covers S12+S22
    LF2_active = s22_dark and s32_dark       # Low flood covers S22+S32

    if not emergency_mode:
        # ----- Normal AUTO control (identical to your main.py math) -----
        # Column 1
        state["HS11"] = min(state["HS11"] + step, 255) if (s11_dark and not LS1_active) else max(state["HS11"] - step, 0)
        state["HS21"] = min(state["HS21"] + step, 255) if (s21_dark and not LS1_active and not LF1_active) else max(state["HS21"] - step, 0)
        state["HF1"]  = min(state["HF1"]  + step, 255) if (s31_dark and not LF1_active) else max(state["HF1"]  - step, 0)
        state["LS1"]  = min(state["LS1"]  + step, 255) if LS1_active else max(state["LS1"]  - step, 0)
        state["LF1"]  = min(state["LF1"]  + step, 255) if LF1_active else max(state["LF1"]  - step, 0)

        # Column 2
        state["HS12"] = min(state["HS12"] + step, 255) if (s12_dark and not LS2_active) else max(state["HS12"] - step, 0)
        state["HS22"] = min(state["HS22"] + step, 255) if (s22_dark and not LS2_active and not LF2_active) else max(state["HS22"] - step, 0)
        state["HF2"]  = min(state["HF2"]  + step, 255) if (s32_dark and not LF2_active) else max(state["HF2"]  - step, 0)
        state["LS2"]  = min(state["LS2"]  + step, 255) if LS2_active else max(state["LS2"]  - step, 0)
        state["LF2"]  = min(state["LF2"]  + step, 255) if LF2_active else max(state["LF2"]  - step, 0)
    else:
        # ----- Emergency override in AUTO -----
        # Zero everything except LF2, which ramps by +15 to max (matches your main.py)
        state["HS11"] = state["HS21"] = state["LS1"] = state["HF1"] = state["LF1"] = 0
        state["HS12"] = state["HS22"] = state["LS2"] = state["HF2"] = 0
        state["LF2"]  = min(state["LF2"] + 15, 255)

    # Push to DACs (column-aware)
    _push_all(state)
    return state
