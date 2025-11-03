#!/usr/bin/env python3
"""
DACBus.py — Two-LTC1665 driver for Raspberry Pi 5 (SPI1 CE1/CE0), with logical light names.

Features
--------
- Column 1 on SPI1 CE1, Column 2 on SPI1 CE0 (matches your existing tests).
- Logical light names mapped to (column, channel) for all 10 lights:
    Col 1: HS11→1, HS21→2, LS1→3, HF1→4, LF1→5
    Col 2: HS12→1, HS22→2, LS2→3, HF2→4, LF2→5
- Batch write (write_all) for deterministic, flicker-free updates.
- Safe clamping (0..255), quick helpers (zero_all, close).
- Optional overrides via Config.py if you add them later.

LTC1665 Frame (per datasheet)
-----------------------------
  Bits 15..12 : channel address (1..8 for A..H)
  Bits 11..4  : 8-bit DAC value
  Bits 3..0   : zeros
  SPI is MSB first.

Dependencies
------------
  sudo apt-get install python3-spidev
  (Your project already uses spidev.)

"""

from dataclasses import dataclass
from typing import Dict, Tuple, Optional
import spidev

# ------------------------- Defaults (match your repo) -------------------------

# RPi 5 secondary SPI controller
DEFAULT_SPI_BUS = 1

# Column 1 -> CE1 (device=1), Column 2 -> CE0 (device=0)
DEFAULT_COL_TO_DEVICE = {1: 1, 2: 0}

# Canonical light mapping (logical → (column, channel))
DEFAULT_CHANNEL_MAP: Dict[str, Tuple[int, int]] = {
    # Column 1
    "HS11": (1, 1),
    "HS21": (1, 2),
    "LS1":  (1, 3),
    "HF1":  (1, 4),
    "LF1":  (1, 5),
    # Column 2
    "HS12": (2, 1),
    "HS22": (2, 2),
    "LS2":  (2, 3),
    "HF2":  (2, 4),
    "LF2":  (2, 5),
}

# Reasonable SPI defaults for LTC1665
DEFAULT_MAX_SPEED_HZ = 1_000_000   # 1 MHz is ample
DEFAULT_MODE = 0                    # CPOL=0, CPHA=0
DEFAULT_BITS_PER_WORD = 8


@dataclass
class _SpiDevHandle:
    """Small wrapper to manage a spidev handle."""
    bus: int
    device: int
    max_speed_hz: int = DEFAULT_MAX_SPEED_HZ
    mode: int = DEFAULT_MODE
    bits_per_word: int = DEFAULT_BITS_PER_WORD
    handle: Optional[spidev.SpiDev] = None

    def ensure_open(self) -> spidev.SpiDev:
        if self.handle is None:
            h = spidev.SpiDev()
            h.open(self.bus, self.device)
            h.max_speed_hz = self.max_speed_hz
            h.mode = self.mode
            h.bits_per_word = self.bits_per_word
            self.handle = h
        return self.handle

    def xfer2(self, data: bytes) -> None:
        h = self.ensure_open()
        # spidev expects a list of ints [0..255]
        h.xfer2(list(data))

    def close(self) -> None:
        if self.handle is not None:
            try:
                self.handle.close()
            finally:
                self.handle = None


class DACBus:
    """
    High-level driver for two LTC1665 DACs acting as Column 1 and Column 2.
    Use `from_config(Config)` to build from your project config, or pass kwargs.

    Example:
        dac = DACBus.from_config(C)
        dac.write_all({"HS11": 120, "LS1": 90, "LF2": 180})
        ...
        dac.zero_all()
        dac.close()
    """

    def __init__(
        self,
        *,
        spi_bus: int = DEFAULT_SPI_BUS,
        col_to_device: Dict[int, int] = None,
        channel_map: Dict[str, Tuple[int, int]] = None,
        max_speed_hz: int = DEFAULT_MAX_SPEED_HZ,
        mode: int = DEFAULT_MODE,
        bits_per_word: int = DEFAULT_BITS_PER_WORD,
    ):
        self.spi_bus = spi_bus
        self.col_to_device = dict(col_to_device or DEFAULT_COL_TO_DEVICE)
        self.channel_map = dict(channel_map or DEFAULT_CHANNEL_MAP)

        # One SpiDev per CE (device)
        self._handles: Dict[int, _SpiDevHandle] = {}
        for col, dev in self.col_to_device.items():
            self._handles[col] = _SpiDevHandle(
                bus=self.spi_bus,
                device=dev,
                max_speed_hz=max_speed_hz,
                mode=mode,
                bits_per_word=bits_per_word,
            )

    # --------------- Constructors ---------------

    @classmethod
    def from_config(cls, ConfigModule):
        """
        Build from modules.Config if present. Falls back to safe defaults.
        You can add any of these optional names to Config.py later:
          SPI_BUS, COL_TO_DEVICE, CHANNEL_MAP, DAC_MAX_SPEED_HZ, DAC_MODE, DAC_BITS
        """
        spi_bus = getattr(ConfigModule, "SPI_BUS", DEFAULT_SPI_BUS)
        col_to_device = getattr(ConfigModule, "COL_TO_DEVICE", DEFAULT_COL_TO_DEVICE)
        channel_map = getattr(ConfigModule, "CHANNEL_MAP", DEFAULT_CHANNEL_MAP)
        max_speed_hz = getattr(ConfigModule, "DAC_MAX_SPEED_HZ", DEFAULT_MAX_SPEED_HZ)
        mode = getattr(ConfigModule, "DAC_MODE", DEFAULT_MODE)
        bits = getattr(ConfigModule, "DAC_BITS", DEFAULT_BITS_PER_WORD)
        return cls(
            spi_bus=spi_bus,
            col_to_device=col_to_device,
            channel_map=channel_map,
            max_speed_hz=max_speed_hz,
            mode=mode,
            bits_per_word=bits,
        )

    # --------------- Public API ---------------

    def write(self, light: str, value: int) -> None:
        """
        Write a single logical light (0..255).
        """
        if light not in self.channel_map:
            raise KeyError(f"Unknown light name '{light}'. Known: {sorted(self.channel_map.keys())}")
        col, ch = self.channel_map[light]
        self._write_channel(value, channel=ch, column=col)

    def write_all(self, values: Dict[str, int]) -> None:
        """
        Batch write multiple lights. This groups writes by column to minimize CE switching.
        Missing lights are ignored; unknown names raise KeyError.
        """
        # Group by column
        per_col: Dict[int, Dict[int, int]] = {1: {}, 2: {}}
        for name, val in values.items():
            if name not in self.channel_map:
                raise KeyError(f"Unknown light name '{name}'. Known: {sorted(self.channel_map.keys())}")
            col, ch = self.channel_map[name]
            per_col[col][ch] = _clamp_byte(val)

        # Push Column 1 then Column 2 (order not critical)
        for col in (1, 2):
            if per_col[col]:
                self._write_many(column=col, ch_to_val=per_col[col])

    def zero_all(self) -> None:
        """
        Turn off all known channels (writes 0) on both columns for channels seen in the map.
        """
        # Build a minimal set of channels per column from the map
        per_col = {1: set(), 2: set()}
        for _, (col, ch) in self.channel_map.items():
            per_col[col].add(ch)

        for col in (1, 2):
            if per_col[col]:
                ch_to_val = {ch: 0 for ch in sorted(per_col[col])}
                self._write_many(column=col, ch_to_val=ch_to_val)

    def close(self) -> None:
        """Close both SPI handles."""
        for h in self._handles.values():
            h.close()

    # --------------- Low-level writers ---------------

    def _write_many(self, *, column: int, ch_to_val: Dict[int, int]) -> None:
        """
        Write several (channel → value) pairs on the same column.
        """
        h = self._handles[column]
        # Emit frames in ascending channel order for determinism
        for ch in sorted(ch_to_val.keys()):
            val = ch_to_val[ch]
            self._xfer_frame(h, ch, val)

    def _write_channel(self, value: int, *, channel: int, column: int) -> None:
        """
        Low-level single write by numeric channel and column.
        """
        value = _clamp_byte(value)
        if channel < 1 or channel > 8:
            raise ValueError("Channel must be 1..8 for LTC1665")
        h = self._handles[column]
        self._xfer_frame(h, channel, value)

    @staticmethod
    def _build_frame(channel: int, value: int) -> bytes:
        """
        Build the two-byte LTC1665 command frame:
            [15:12]=channel, [11:4]=data(8), [3:0]=0
        """
        cmd = (channel << 12) | (value << 4)
        hi = (cmd >> 8) & 0xFF
        lo = (cmd >> 0) & 0xFF
        return bytes((hi, lo))

    def _xfer_frame(self, handle: _SpiDevHandle, channel: int, value: int) -> None:
        frame = self._build_frame(channel, value)
        handle.xfer2(frame)


# ----------------------------- Utility helpers -----------------------------

def _clamp_byte(v: int) -> int:
    try:
        iv = int(v)
    except Exception:
        iv = 0
    if iv < 0:   return 0
    if iv > 255: return 255
    return iv
