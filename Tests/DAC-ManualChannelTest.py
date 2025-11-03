#!/usr/bin/env python3
# DAC-ManualChannelTest.py — two-column test (SPI1 CE0/CE1)

import time
import spidev

# --- Hardware mapping (RPi 5) ---
# Column 1 -> SPI1, CE1 -> GPIO17 (pin 11)
# Column 2 -> SPI1, CE0 -> GPIO18 (pin 12)
SPI_BUS = 1
COL_TO_DEVICE = {1: 1, 2: 0}  # Col1=CE1, Col2=CE0

# LTC1665 command format:
#  - Bits 15..12: channel address (1..8 for A..H)
#  - Bits 11..4 : 8-bit DAC value
#  - Bits 3..0  : zeros
def write_dac(value: int, channel: int, column: int):
    if channel < 1 or channel > 8:
        raise ValueError("Channel must be 1..8 (A..H).")
    if value < 0 or value > 255:
        raise ValueError("Value must be 0..255.")
    if column not in (1, 2):
        raise ValueError("Column must be 1 or 2.")

    cmd = (channel << 12) | (value << 4)
    hi = (cmd >> 8) & 0xFF
    lo = cmd & 0xFF

    dev = COL_TO_DEVICE[column]
    spi = spidev.SpiDev()
    spi.open(SPI_BUS, dev)     # (bus=1, device=0 or 1)
    spi.max_speed_hz = 2_000_000
    spi.mode = 0               # LTC1665 uses SPI Mode 0
    spi.writebytes([hi, lo])
    spi.close()

# Map channel letters A–H to 1–8
CHANNEL_MAP = {c: i+1 for i, c in enumerate("abcdefgh")}

def main():
    print("Two-Column DAC Manual Test (SPI1)")
    print("  Column 1 -> SPI1 CE1 (GPIO17 / pin 11)")
    print("  Column 2 -> SPI1 CE0 (GPIO18 / pin 12)")
    print("Tips: type 's' at the prompt to switch columns, or 'q' to quit.\n")

    active_col = 1  # default to Column 1 on CE1

    while True:
        print(f"\nActive Column: {active_col}  (1=CE1, 2=CE0)")
        ch = input("Enter channel (A–H), 's' to change column, or 'q' to quit: ").strip().lower()

        if ch == 'q':
            print("Exiting DAC test.")
            break

        if ch == 's':
            # quick toggle
            active_col = 2 if active_col == 1 else 1
            print(f"Switched active column to {active_col}.")
            continue

        if ch not in CHANNEL_MAP:
            print("  Invalid input. Please enter A–H, 's' to switch columns, or 'q' to quit.")
            continue

        # Optionally override column for this test
        col_input = input(f"Press Enter to use Column {active_col}, or type 1/2 to override: ").strip()
        if col_input in ("1", "2"):
            test_col = int(col_input)
        elif col_input == "":
            test_col = active_col
        else:
            print("  Invalid input. Using active column.")
            test_col = active_col

        ch_num = CHANNEL_MAP[ch]
        print(f"\n[Column {test_col}] Setting channel {ch.upper()} (#{ch_num}) to 150 for 2 seconds...")
        write_dac(150, ch_num, test_col)
        time.sleep(2)
        print(f"[Column {test_col}] Turning channel {ch.upper()} (#{ch_num}) off.")
        write_dac(0, ch_num, test_col)

if __name__ == "__main__":
    main()
