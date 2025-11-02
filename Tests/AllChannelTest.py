#!/usr/bin/env python3
# AllChannelTest.py — two DAC columns (SPI1) with interactive patterns
# Column 1 -> SPI1 CE1 (GPIO17 / pin 11)
# Column 2 -> SPI1 CE0 (GPIO18 / pin 12)

import time
import spidev
import glob

# ---- Hardware mapping (RPi 5) ----
SPI_BUS = 1
COL_TO_DEVICE = {1: 1, 2: 0}  # Column 1 = CE1, Column 2 = CE0

# Channel letters A–H map to 1..8 (LTC1665 uses addresses 1..8)
CHANNELS = [("A", 1), ("B", 2), ("C", 3), ("D", 4),
            ("E", 5), ("F", 6), ("G", 7), ("H", 8)]

def _open_spi(device):
    spi = spidev.SpiDev()
    try:
        spi.open(SPI_BUS, device)  # (1, 1) for CE1, (1, 0) for CE0
    except FileNotFoundError:
        available = " ".join(sorted(glob.glob("/dev/spidev*"))) or "(none)"
        raise FileNotFoundError(
            f"/dev/spidev{SPI_BUS}.{device} not found.\n"
            "Enable SPI1 with two chip selects by adding this to /boot/firmware/config.txt:\n"
            "  dtoverlay=spi1-2cs,cs0_pin=18,cs1_pin=17\n"
            f"Currently available: {available}"
        )
    spi.max_speed_hz = 2_000_000
    spi.mode = 0
    return spi

def write_dac(value: int, channel: int, column: int):
    if channel < 1 or channel > 8:
        raise ValueError("Channel must be 1..8 (A..H).")
    if not (0 <= value <= 255):
        raise ValueError("Value must be 0..255.")
    if column not in (1, 2):
        raise ValueError("Column must be 1 or 2.")

    cmd = (channel << 12) | (value << 4)
    hi = (cmd >> 8) & 0xFF
    lo = cmd & 0xFF

    dev = COL_TO_DEVICE[column]
    spi = _open_spi(dev)
    spi.writebytes([hi, lo])
    spi.close()

def set_all_channels(value: int, column: int):
    for _, ch_num in CHANNELS:
        write_dac(value, ch_num, column)

def one_hot_cycle(column: int, value: int = 150, dwell_s: float = 1.0):
    print(f"\n[Col {column}] One-hot cycle A→H @ {value} for {dwell_s:.2f}s each.")
    for name, ch_num in CHANNELS:
        print(f"[Col {column}] {name} -> {value}")
        write_dac(value, ch_num, column)
        time.sleep(dwell_s)
        write_dac(0, ch_num, column)

def all_on_flash(column: int, value: int = 150, dwell_s: float = 1.0):
    print(f"\n[Col {column}] ALL ON @ {value} for {dwell_s:.2f}s, then OFF.")
    set_all_channels(value, column)
    time.sleep(dwell_s)
    set_all_channels(0, column)

def ramp_each_channel(column: int, step: int = 15, delay_s: float = 0.03):
    step = max(1, min(step, 255))
    print(f"\n[Col {column}] Ramp each channel 0→255→0 (step {step}, delay {delay_s:.3f}s).")
    for name, ch_num in CHANNELS:
        print(f"[Col {column}] Ramping {name} up")
        for v in range(0, 256, step):
            write_dac(v, ch_num, column)
            time.sleep(delay_s)
        print(f"[Col {column}] Ramping {name} down")
        for v in range(255, -1, -step):
            write_dac(v, ch_num, column)
            time.sleep(delay_s)
        write_dac(0, ch_num, column)

def choose_columns() -> list[int]:
    while True:
        s = input("Select column: [1=CE1, 2=CE0, b=both]: ").strip().lower()
        if s == "1":
            return [1]
        if s == "2":
            return [2]
        if s in ("b", "both"):
            return [1, 2]
        print("  Invalid input. Enter 1, 2, or b.")

def main():
    print("AllChannelTest — two DAC columns on SPI1")
    print("  Column 1 -> SPI1 CE1 (GPIO17 / pin 11)")
    print("  Column 2 -> SPI1 CE0 (GPIO18 / pin 12)")
    print("Patterns:")
    print("  1) One-hot cycle (A→H)")
    print("  2) All-on flash")
    print("  3) Ramp each channel")
    print("  4) All OFF")
    print("  q) Quit\n")

    while True:
        choice = input("Select pattern [1/2/3/4/q]: ").strip().lower()
        if choice == "q":
            print("Goodbye.")
            break

        cols = choose_columns()

        if choice == "1":
            try:
                v = input("Value 0–255 (Enter=150): ").strip()
                v = 150 if v == "" else max(0, min(255, int(v)))
                d = input("Dwell seconds per channel (Enter=1.0): ").strip()
                d = 1.0 if d == "" else max(0.0, float(d))
            except ValueError:
                print("  Invalid entry; using defaults (150, 1.0s).")
                v, d = 150, 1.0
            for c in cols:
                one_hot_cycle(c, v, d)

        elif choice == "2":
            try:
                v = input("Value 0–255 (Enter=150): ").strip()
                v = 150 if v == "" else max(0, min(255, int(v)))
                d = input("Dwell seconds (Enter=1.0): ").strip()
                d = 1.0 if d == "" else max(0.0, float(d))
            except ValueError:
                print("  Invalid entry; using defaults (150, 1.0s).")
                v, d = 150, 1.0
            for c in cols:
                all_on_flash(c, v, d)

        elif choice == "3":
            try:
                st = input("Step size 1–255 (Enter=15): ").strip()
                st = 15 if st == "" else max(1, min(255, int(st)))
                dl = input("Delay per step in seconds (Enter=0.03): ").strip()
                dl = 0.03 if dl == "" else max(0.0, float(dl))
            except ValueError:
                print("  Invalid entry; using defaults (step=15, delay=0.03s).")
                st, dl = 15, 0.03
            for c in cols:
                ramp_each_channel(c, st, dl)

        elif choice == "4":
            for c in cols:
                print(f"[Col {c}] All OFF")
                set_all_channels(0, c)

        else:
            print("  Invalid selection. Choose 1, 2, 3, 4, or q.")

if __name__ == "__main__":
    main()
