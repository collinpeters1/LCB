
#!/usr/bin/env python3
#AimAndTune.py
import cv2
import time
import spidev
import numpy as np

from modules.LightingAnalysis import CameraManager, LightingAnalyzer

# ---------------- DAC interface ----------------
SPI_BUS = 1
# Column 1 is CE1 -> device 1, Column 2 is CE0 -> device 0 (your wiring)
COL_TO_DEVICE = {1: 1, 2: 0}
SPI_MAX_HZ = 2_000_000

CHANNELS = ["A", "B", "C", "D", "E"]  # logical names
NUM_CHANNELS = len(CHANNELS)

class DAC:
    """
    Lightweight SPI driver for two LTC1665 chips (one per column).
    We only care about channels 1..5 (A..E).
    """
    def __init__(self):
        self.devs = {}
        for col, dev in COL_TO_DEVICE.items():
            s = spidev.SpiDev()
            s.open(SPI_BUS, dev)
            s.max_speed_hz = SPI_MAX_HZ
            s.mode = 0
            self.devs[col] = s

    def close(self):
        for s in self.devs.values():
            try:
                s.close()
            except:
                pass

    def write(self, column: int, channel: int, value: int):
        """
        channel: 1..8 for LTC1665
        value: 0..255
        Frame format:
            [15:12] channel
            [11:4]  value
            [3:0]   0
        """
        if channel < 1 or channel > 8:
            return
        v = 0 if value < 0 else 255 if value > 255 else int(value)
        cmd = (channel << 12) | (v << 4)
        hi = (cmd >> 8) & 0xFF
        lo = cmd & 0xFF
        self.devs[column].writebytes([hi, lo])

    def write_all(self, state):
        """
        state[col][ch] = value
        ch is 1..5 here
        """
        for col in (1, 2):
            for ch in range(1, 6):
                self.write(col, ch, state[col][ch])

def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

# ---------------- HUD overlay helpers ----------------
def draw_hud(annotated_img, active_col, active_idx, state, analyzer):
    """
    Draws text info about:
    - DAC column and channel currently selected
    - Current DAC values for both columns
    - Threshold mode / value
    over the analyzed/annotated frame.
    """
    hud_lines = []

    # Active selection
    hud_lines.append(
        f"COLUMN {active_col} | CH {CHANNELS[active_idx]} ({active_idx+1})"
    )

    # Values table summary
    # ex: Col1 [A:064 B:000 C:255 D:010 E:000]
    col1_vals = " ".join(
        f"{CHANNELS[i]}:{state[1][i+1]:03d}" for i in range(NUM_CHANNELS)
    )
    col2_vals = " ".join(
        f"{CHANNELS[i]}:{state[2][i+1]:03d}" for i in range(NUM_CHANNELS)
    )
    hud_lines.append(f"Col1 [{col1_vals}]")
    hud_lines.append(f"Col2 [{col2_vals}]")

    # Threshold / mode info from analyzer
    hud_lines.append(
        f"THRESH_MODE={analyzer.threshold_mode}  THRESH={analyzer.get_threshold()}"
    )

    # Controls reminder (short version so it fits)
    hud_lines.append("1/2 col  a/d chan  w/W +1/+25  s/S -1/-25  space toggle")
    hud_lines.append("[ ] thr  \\ mode  x/X zero 0 off = max  q quit")

    x0 = 10
    y0 = 20
    for i, text in enumerate(hud_lines):
        y = y0 + i * 20
        # shadow
        cv2.putText(
            annotated_img, text, (x0+1, y+1),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (0, 0, 0), 2, cv2.LINE_AA
        )
        # main
        cv2.putText(
            annotated_img, text, (x0, y),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (255, 255, 255), 1, cv2.LINE_AA
        )

# ---------------- main loop ----------------
def main():
    # camera + analysis
    cam = CameraManager(camera_index=0)
    analyzer = LightingAnalyzer(threshold_value=40, rows=3, cols=2)

    # dac state
    dac = DAC()
    state = {
        1: {ch: 0 for ch in range(1, 6)},  # column 1, ch1..5
        2: {ch: 0 for ch in range(1, 6)}   # column 2, ch1..5
    }
    last_nonzero = {
        1: {ch: 64 for ch in range(1, 6)},
        2: {ch: 64 for ch in range(1, 6)},
    }

    # initialize outputs off
    dac.write_all(state)

    active_col = 1
    active_idx = 0  # 0=A .. 4=E

    print("Aiming / Tuning Tool Running")
    print("Window must be focused for keypresses to register.")
    print("Keys:")
    print("  1 / 2         select column 1 or 2")
    print("  a / d         previous / next channel (A..E)")
    print("  w / +         +1 on current channel")
    print("  W             +25 on current channel")
    print("  s / -         -1 on current channel")
    print("  S             -25 on current channel")
    print("  0             set current channel = 0")
    print("  =             set current channel = 255")
    print("  space         toggle 0 <-> last non-zero for this channel")
    print("  x             zero all channels on active column")
    print("  X             zero both columns")
    print("  [ / ]         threshold -5 / +5")
    print("  \\             toggle threshold_mode global <-> local_otsu")
    print("  q             quit\n")

    try:
        while True:
            frame = cam.capture_frame()
            if frame is None:
                # no frame this iteration
                time.sleep(0.01)
                continue

            overall_dark, cell_darkness, annotated_img = analyzer.analyze(frame)

            # annotate darkness numbers in console (optional)
            # you can comment this out if it's too spammy live
            print(
                f"Overall {overall_dark:.1f}% | "
                f"S11={cell_darkness[0]:.1f} "
                f"S12={cell_darkness[1]:.1f} "
                f"S21={cell_darkness[2]:.1f} "
                f"S22={cell_darkness[3]:.1f} "
                f"S31={cell_darkness[4]:.1f} "
                f"S32={cell_darkness[5]:.1f}   ",
                end="\r"
            )

            # draw HUD with DAC info and controls
            draw_hud(annotated_img, active_col, active_idx, state, analyzer)

            # show video feeds
            cv2.imshow("AIMING VIEW (Analysis Overlay)", annotated_img)
            cv2.imshow("RAW CAMERA", frame)

            # handle keys
            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                break

            # select column 1 or 2
            elif key == ord('1'):
                active_col = 1
            elif key == ord('2'):
                active_col = 2

            # move between channels A..E
            elif key == ord('a'):  # prev
                active_idx = (active_idx - 1) % NUM_CHANNELS
            elif key == ord('d'):  # next
                active_idx = (active_idx + 1) % NUM_CHANNELS

            # up small (+1)
            elif key in (ord('w'), ord('+')):
                ch_num = active_idx + 1
                state[active_col][ch_num] = clamp(state[active_col][ch_num] + 1, 0, 255)
                dac.write(active_col, ch_num, state[active_col][ch_num])
                if state[active_col][ch_num] > 0:
                    last_nonzero[active_col][ch_num] = state[active_col][ch_num]

            # up big (+25)
            elif key == ord('W'):
                ch_num = active_idx + 1
                state[active_col][ch_num] = clamp(state[active_col][ch_num] + 25, 0, 255)
                dac.write(active_col, ch_num, state[active_col][ch_num])
                if state[active_col][ch_num] > 0:
                    last_nonzero[active_col][ch_num] = state[active_col][ch_num]

            # down small (-1)
            elif key in (ord('s'), ord('-')):
                ch_num = active_idx + 1
                state[active_col][ch_num] = clamp(state[active_col][ch_num] - 1, 0, 255)
                dac.write(active_col, ch_num, state[active_col][ch_num])
                if state[active_col][ch_num] > 0:
                    last_nonzero[active_col][ch_num] = state[active_col][ch_num]

            # down big (-25)
            elif key == ord('S'):
                ch_num = active_idx + 1
                state[active_col][ch_num] = clamp(state[active_col][ch_num] - 25, 0, 255)
                dac.write(active_col, ch_num, state[active_col][ch_num])
                if state[active_col][ch_num] > 0:
                    last_nonzero[active_col][ch_num] = state[active_col][ch_num]

            # set 0
            elif key == ord('0'):
                ch_num = active_idx + 1
                state[active_col][ch_num] = 0
                dac.write(active_col, ch_num, 0)

            # set 255
            elif key == ord('='):
                ch_num = active_idx + 1
                state[active_col][ch_num] = 255
                last_nonzero[active_col][ch_num] = 255
                dac.write(active_col, ch_num, 255)

            # toggle 0 <-> last_nonzero
            elif key == ord(' '):
                ch_num = active_idx + 1
                if state[active_col][ch_num] == 0:
                    v = last_nonzero[active_col][ch_num] or 64
                    state[active_col][ch_num] = v
                    dac.write(active_col, ch_num, v)
                else:
                    last_nonzero[active_col][ch_num] = state[active_col][ch_num]
                    state[active_col][ch_num] = 0
                    dac.write(active_col, ch_num, 0)

            # zero active column
            elif key == ord('x'):
                for ch_num in range(1, 6):
                    state[active_col][ch_num] = 0
                dac.write_all(state)

            # zero both columns
            elif key == ord('X'):
                for col in (1, 2):
                    for ch_num in range(1, 6):
                        state[col][ch_num] = 0
                dac.write_all(state)

            # lighting analysis tuning
            elif key == ord('['):
                analyzer.adjust_threshold(-5)
            elif key == ord(']'):
                analyzer.adjust_threshold(+5)
            elif key == ord('\\'):
                # toggle global/local_otsu thresholding
                new_mode = "local_otsu" if analyzer.threshold_mode == "global" else "global"
                analyzer.set_threshold_mode(new_mode)

            # loop timing
            time.sleep(0.01)

    finally:
        # safety: turn off lights before exit (change if you DON'T want that)
        for col in (1, 2):
            for ch_num in range(1, 6):
                state[col][ch_num] = 0
        dac.write_all(state)

        dac.close()

        try:
            cam.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == "__main__":
    main()
