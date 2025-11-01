#!/usr/bin/env python3
# ------------------------------------------------------------
# DAC-ManualChannelTest.py
#
# Purpose:
#   A fast, keyboard-based test utility for manually controlling
#   two LTC1665 DAC chips (8-channel 8-bit DACs) connected to
#   a Raspberry Pi 5 over SPI1 (one DAC per lighting column).
#
#   It lets you quickly adjust output values (0–255) for channels
#   A–E only (you don’t need F–H), switch between columns, and
#   instantly write values to the DAC for testing lights.
#
#   No mouse required — everything is done from the keyboard.
#
# ------------------------------------------------------------
# Keyboard controls:
#   Tab / s        : switch between column 1 ↔ 2
#   ← / →          : move to previous/next channel (A..E)
#   ↑ / +          : increase current channel value by 1
#   ↓ / -          : decrease current channel value by 1
#   PageUp         : increase by 16  (coarse up)
#   PageDown       : decrease by 16  (coarse down)
#   0              : set value to 0
#   =              : set value to 255 (max)
#   v              : manually type a numeric value 0–255
#   space          : toggle between 0 and last non-zero value
#   x              : zero all channels on the active column
#   X              : zero all channels on both columns
#   q              : quit (optionally clears outputs if enabled)
# ------------------------------------------------------------

import curses      # For building the interactive terminal interface
import spidev      # For SPI communication with the DAC chips
import time

# ---------- Configuration section ----------
SPI_BUS = 1
# Column 1 is wired to CE1, Column 2 to CE0 on SPI1
COL_TO_DEVICE = {1: 1, 2: 0}         # CE1 → device 1, CE0 → device 0
SPI_MAX_HZ = 2_000_000               # SPI speed — 2 MHz is safe and reliable
ZERO_ON_EXIT = False                 # Set True if you want outputs to go to 0 on quit
CHANNELS = ["A", "B", "C", "D", "E"] # Only use 5 channels (A–E)
# Create a mapping:  "A"→1, "B"→2, ... "E"→5
CH_TO_NUM = {c: i+1 for i, c in enumerate(CHANNELS)}

# ---------- DAC SPI interface class ----------
class DAC:
    """Handles SPI communication with both DAC chips."""
    def __init__(self):
        # Open both SPI devices (one per column)
        self.devs = {}
        for col, dev in COL_TO_DEVICE.items():
            s = spidev.SpiDev()
            s.open(SPI_BUS, dev)
            s.max_speed_hz = SPI_MAX_HZ
            s.mode = 0  # SPI mode 0 works for LTC1665
            self.devs[col] = s

    def close(self):
        """Close all SPI device handles safely."""
        for s in self.devs.values():
            try:
                s.close()
            except:
                pass

    def write(self, *, column: int, channel: int, value: int):
        """
        Send an 8-bit value (0–255) to one DAC channel.

        Each LTC1665 command frame = 16 bits:
          bits 15–12 : channel address (1–8)
          bits 11–4  : 8-bit DAC value
          bits 3–0   : unused zeros
        """
        v = 0 if value < 0 else 255 if value > 255 else int(value)
        cmd = (channel << 12) | (v << 4)
        hi = (cmd >> 8) & 0xFF
        lo = cmd & 0xFF
        self.devs[column].writebytes([hi, lo])

    def write_all(self, state):
        """
        Push all stored values to both DACs at once.

        state must be a nested dict like:
        {1: {1:v,2:v,3:v,4:v,5:v}, 2: {...}}
        """
        for col in (1, 2):
            for ch in range(1, 6):
                self.write(column=col, channel=ch, value=state[col][ch])

# ---------- Helper functions ----------
def clamp(v, lo, hi):
    """Clamp a value within a specified range."""
    return lo if v < lo else hi if v > hi else v

# Draws the entire text interface on screen
def draw_ui(stdscr, active_col, active_idx, state, last_nonzero):
    stdscr.clear()
    h, w = stdscr.getmaxyx()  # Get current terminal size

    # Title centered at the top
    title = "Two-Column DAC Quick Test  —  A–E only (LTC1665)"
    stdscr.addstr(0, max(0, (w - len(title)) // 2), title, curses.A_BOLD)

    # Current column indicator
    stdscr.addstr(2, 2, f"Active Column: {active_col}  (Tab/s to switch)")

    # Channel selector row (A..E)
    stdscr.addstr(4, 2, "Channels: ")
    base_x = 12
    for i, ch in enumerate(CHANNELS):
        label = f" {ch} "
        attr = curses.A_REVERSE if i == active_idx else curses.A_NORMAL
        stdscr.addstr(4, base_x + i*4, label, attr)

    # Display numeric DAC values for each column
    stdscr.addstr(6, 2, "Values (0..255):")
    stdscr.addstr(7, 4, "Col 1:", curses.A_BOLD)
    stdscr.addstr(8, 4, "Col 2:", curses.A_BOLD)

    for i, ch in enumerate(CHANNELS):
        x = 12 + i*6
        v1 = state[1][i+1]
        v2 = state[2][i+1]
        attr1 = curses.A_REVERSE if (active_col == 1 and i == active_idx) else curses.A_NORMAL
        attr2 = curses.A_REVERSE if (active_col == 2 and i == active_idx) else curses.A_NORMAL
        stdscr.addstr(7, x, f"{v1:3d}", attr1)
        stdscr.addstr(8, x, f"{v2:3d}", attr2)

    # Quick-reference help text below the table
    help_lines = [
        "←/→: prev/next channel    ↑/+/PageUp: +1/+16    ↓/-/PageDown: -1/-16",
        "Tab/s: switch column      0: zero     =: 255     space: toggle 0 <-> last",
        "v: type value (0..255)    x: zero active column  X: zero BOTH columns",
        "q: quit",
    ]
    start_y = 10
    for i, line in enumerate(help_lines):
        stdscr.addstr(start_y + i, 2, line)

    stdscr.refresh()

# Ask the user to type a value (0–255) in the terminal
def prompt_value(stdscr, prompt="Enter value 0..255: "):
    curses.echo()  # show typed characters
    stdscr.addstr(14, 2, " " * 50)
    stdscr.addstr(14, 2, prompt)
    stdscr.refresh()
    try:
        s = stdscr.getstr(14, 2 + len(prompt), 4).decode("utf-8").strip()
    except Exception:
        s = ""
    curses.noecho()
    try:
        v = int(s)
        return clamp(v, 0, 255)
    except Exception:
        return None

# ---------- Main interactive loop ----------
def main(stdscr):
    # Configure curses terminal behavior
    curses.curs_set(0)        # hide text cursor
    stdscr.nodelay(False)     # wait for key presses
    stdscr.keypad(True)       # enable arrow keys, page keys, etc.

    dac = DAC()  # create the DAC interface object
    try:
        # Initialize per-column per-channel value tables
        # state[col][ch] = current output value
        state = {1: {ch: 0 for ch in range(1, 6)},
                 2: {ch: 0 for ch in range(1, 6)}}

        # last_nonzero remembers the last non-zero value
        # so we can toggle back to it with the spacebar
        last_nonzero = {1: {ch: 64 for ch in range(1, 6)},
                        2: {ch: 64 for ch in range(1, 6)}}

        active_col = 1   # start with column 1 selected
        active_idx = 0   # index 0 = channel A
        dac.write_all(state)  # set everything to 0 on start

        draw_ui(stdscr, active_col, active_idx, state, last_nonzero)

        # -------- Main key-handling loop --------
        while True:
            ch = stdscr.getch()  # read one key

            # --- Exit program ---
            if ch in (ord('q'), 27):  # 'q' or ESC
                if ZERO_ON_EXIT:
                    for c in (1, 2):
                        for n in range(1, 6):
                            state[c][n] = 0
                    dac.write_all(state)
                break

            # --- Navigation keys ---
            elif ch in (curses.KEY_LEFT, ord('h')):
                active_idx = (active_idx - 1) % len(CHANNELS)
            elif ch in (curses.KEY_RIGHT, ord('l')):
                active_idx = (active_idx + 1) % len(CHANNELS)
            elif ch in (curses.KEY_BTAB, curses.KEY_STAB, ord('\t'), ord('s'), ord('S')):
                # Switch between column 1 and 2
                active_col = 2 if active_col == 1 else 1

            # --- Fine adjustments ---
            elif ch in (curses.KEY_UP, ord('+')):
                chan = active_idx + 1
                state[active_col][chan] = clamp(state[active_col][chan] + 1, 0, 255)
                dac.write(column=active_col, channel=chan, value=state[active_col][chan])
            elif ch in (curses.KEY_DOWN, ord('-')):
                chan = active_idx + 1
                state[active_col][chan] = clamp(state[active_col][chan] - 1, 0, 255)
                dac.write(column=active_col, channel=chan, value=state[active_col][chan])

            # --- Coarse adjustments (PageUp/PageDown) ---
            elif ch == curses.KEY_PPAGE:
                chan = active_idx + 1
                state[active_col][chan] = clamp(state[active_col][chan] + 16, 0, 255)
                dac.write(column=active_col, channel=chan, value=state[active_col][chan])
            elif ch == curses.KEY_NPAGE:
                chan = active_idx + 1
                state[active_col][chan] = clamp(state[active_col][chan] - 16, 0, 255)
                dac.write(column=active_col, channel=chan, value=state[active_col][chan])

            # --- Preset buttons ---
            elif ch == ord('0'):
                chan = active_idx + 1
                state[active_col][chan] = 0
                dac.write(column=active_col, channel=chan, value=0)
            elif ch in (ord('='),):
                chan = active_idx + 1
                state[active_col][chan] = 255
                last_nonzero[active_col][chan] = 255
                dac.write(column=active_col, channel=chan, value=255)

            # --- Toggle between 0 and last non-zero value ---
            elif ch == ord(' '):
                chan = active_idx + 1
                if state[active_col][chan] == 0:
                    v = last_nonzero[active_col][chan] or 64
                    state[active_col][chan] = v
                else:
                    last_nonzero[active_col][chan] = state[active_col][chan]
                    state[active_col][chan] = 0
                dac.write(column=active_col, channel=chan, value=state[active_col][chan])

            # --- Zero multiple channels quickly ---
            elif ch in (ord('x'),):
                # zero active column
                for n in range(1, 6):
                    state[active_col][n] = 0
                dac.write_all(state)
            elif ch in (ord('X'),):
                # zero both columns
                for c in (1, 2):
                    for n in range(1, 6):
                        state[c][n] = 0
                dac.write_all(state)

            # --- Manually type a numeric value ---
            elif ch in (ord('v'),):
                v = prompt_value(stdscr)
                if v is not None:
                    chan = active_idx + 1
                    state[active_col][chan] = v
                    if v > 0:
                        last_nonzero[active_col][chan] = v
                    dac.write(column=active_col, channel=chan, value=v)

            # Redraw the interface after every change
            draw_ui(stdscr, active_col, active_idx, state, last_nonzero)
            time.sleep(0.005)  # small delay for smoother updates

    finally:
        # Always close SPI properly even if something crashes
        dac.close()

# ---------- Program entry point ----------
if __name__ == "__main__":
    # curses.wrapper handles terminal setup/cleanup automatically
    curses.wrapper(main)
