
# spi_dac.py
# Hardware adapter for LTC1665 DACs via spidev.
# Uses the same channel map and SPI wiring as your existing LightPolicy.
#
# Public:
#   NAMES              : order of 10 logical channels (matches B columns)
#   CHANNEL_MAP        : logical name -> (column, channel)
#   vec_to_dict(vec)   : 10-vector -> dict{name:int}
#   apply_dict(d)      : write all values
#   apply_vec(v)       : write vector using NAMES order
#   zero_all()         : set all mapped channels to 0
#
# SPI assumptions:
#   bus=1; CE1 = Column 1, CE0 = Column 2
#   LTC1665 command: CCCC DDDDDDDD 0000  (C=1..8 channel code, D=data)
from __future__ import annotations
import spidev

SPI_BUS = 1
COL_TO_DEVICE = {1: 1, 2: 0}  # col -> CE device
DAC_MIN, DAC_MAX = 0, 255

# Logical order we will use everywhere (columns of B must match this!)
NAMES = ["HS11","HS21","LS1","HF1","LF1",
         "HS12","HS22","LS2","HF2","LF2"]

# Map logical name -> (column, channel)  # channels 1..8 on LTC1665
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
    if not (1 <= channel <= 8): raise ValueError("Channel must be 1..8")
    if not (0 <= value <= 255): raise ValueError("Value must be 0..255")
    if column not in (1, 2):    raise ValueError("Column must be 1 or 2")
    cmd = (channel << 12) | (int(value) << 4)  # LTC1665: CCCC DDDDDDDD 0000
    hi = (cmd >> 8) & 0xFF; lo = cmd & 0xFF
    dev = COL_TO_DEVICE[column]
    spi = spidev.SpiDev(); spi.open(SPI_BUS, dev)
    spi.max_speed_hz = 2_000_000; spi.mode = 0
    spi.writebytes([hi, lo])
    spi.close()

def apply_dict(values: dict[str,int]):
    """Write all mapped channels in 'values' dict (clamped 0..255)."""
    for name, raw in values.items():
        if name not in CHANNEL_MAP: continue
        col, ch = CHANNEL_MAP[name]
        v = int(max(DAC_MIN, min(DAC_MAX, int(raw))))
        _write_dac(v, ch, column=col)

def apply_vec(vec):
    """Write a 10-vector with order 'NAMES'."""
    if len(vec) != len(NAMES):
        raise ValueError(f"vec must have len {len(NAMES)}")
    d = {n: int(max(DAC_MIN, min(DAC_MAX, int(v)))) for n, v in zip(NAMES, vec)}
    apply_dict(d)

def zero_all():
    """Zero every hardware-mapped channel."""
    for name, (col, ch) in CHANNEL_MAP.items():
        _write_dac(0, ch, column=col)

def vec_to_dict(vec):
    if len(vec) != len(NAMES):
        raise ValueError(f"vec must have len {len(NAMES)}")
    return {n:int(max(DAC_MIN, min(DAC_MAX, int(v)))) for n, v in zip(NAMES, vec)}
