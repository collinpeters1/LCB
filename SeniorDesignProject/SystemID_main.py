#!/usr/bin/env python3
"""
SystemID_main.py
Fast, mono-camera system-identification test for ICLS lighting.

- Uses Config.ANALYSIS_CAMERA_INDEX to open the correct /dev/videoX
- Forces mono (Y8) capture for the OV9281 analysis cam
- Generates a random-telegraph stimulus (high/low DAC)
- Logs rich per-frame data with metadata header (now logs 6 sector metrics + 6 sector means)
- Estimates delay via *causal* correlation (|corr|, lags >= 0) using S11 only for y[]
- NEW: Runs every mapped light sequentially; zeros all lights between tests
- NEW: Labels logs with light name, column, and channel for MATLAB post-processing
"""

import os, sys, time, csv, signal, glob, subprocess
from datetime import datetime
import numpy as np
import cv2
from scipy.signal import correlate
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ------------------------------------------------------------
# Import project modules
# ------------------------------------------------------------
THIS_DIR = os.path.dirname(os.path.abspath(__file__))
MODULES_DIR = os.path.join(THIS_DIR, "modules")
sys.path.insert(0, THIS_DIR)
sys.path.insert(0, MODULES_DIR)

from modules import PIDPolicy
from modules import Config as C
from modules import ExposureControl as EC
from modules.LightingAnalysis import LightingAnalyzer  # sector %dark

# ------------------------------------------------------------
# CONFIG (globals the menu can tweak)
# ------------------------------------------------------------
RUN_SECONDS = 12.0           # default test duration
STIM_HOLD_FRAMES = 3         # how many frames each bit is held
HIGH_VAL = 200               # DAC high level
LOW_VAL  = 0                 # DAC low level
DARK_THRESH = 20             # pixel < thresh counted as "dark"
# default single-light selection for "Run one light" menu option:
TEST_LIGHT_NAME = "HS22"

CAM_INDEX = int(C.ANALYSIS_CAMERA_INDEX)  # from your Config module
CAM_DEVICE = f"/dev/video{CAM_INDEX}"

# ------------------------------------------------------------
# SIGINT handler
# ------------------------------------------------------------
run_flag = True
def handle_sigint(sig, frame):
    global run_flag
    run_flag = False
    print("\n🛑 Terminating test early...", flush=True)
signal.signal(signal.SIGINT, handle_sigint)

# ------------------------------------------------------------
# DAC helpers
# ------------------------------------------------------------
def write_test_light(light_name: str, raw_val: int):
    v = int(max(0, min(255, raw_val)))
    col, ch = PIDPolicy.CHANNEL_MAP[light_name]
    PIDPolicy._write_dac(v, ch, col)

def zero_all_lights():
    """
    Force ALL DAC outputs on both LTC1665 chips to 0 so every light goes dark.
    SPI1, CE1 = column 1
    SPI1, CE0 = column 2
    """
    print("[DAC] Zeroing all channels on both LTC1665 chips...")
    try:
        import spidev
        spi = spidev.SpiDev()

        # Column 1 (bus 1, device 1)
        spi.open(1, 1)
        spi.max_speed_hz = 2_000_000
        for ch in range(8):  # channels A..H -> 0..7
            high = (ch << 4) | 0x00
            low  = 0x00
            spi.xfer2([high, low])
        spi.close()

        # Column 2 (bus 1, device 0)
        spi.open(1, 0)
        spi.max_speed_hz = 2_000_000
        for ch in range(8):
            high = (ch << 4) | 0x00
            low  = 0x00
            spi.xfer2([high, low])
        spi.close()

        print("[DAC] All channels set to 0.")
    except Exception as e:
        print(f"[ERR] Failed to zero lights: {e}")

# ------------------------------------------------------------
# Frame reshape helpers & stimulus
# ------------------------------------------------------------
def reshape_if_flat_ov9281(frame):
    """
    Handle weird OV9281 mono buffers that come in flattened.
    We try to coerce them into (H, W).
    """
    # Case A: shape like (1, N)
    if frame.ndim == 2 and frame.shape[0] == 1 and frame.shape[1] > 1_000_000:
        n = frame.shape[1]
        if n == 921600:      # 1280 x 720
            return frame.reshape((720, 1280))
        elif n == 1843200:   # 2560 x 720
            return frame.reshape((720, 2560))
        elif n == 2073600:   # 1920 x 1080
            return frame.reshape((1080, 1920))
        else:
            h = 720
            w = n // h
            return frame.reshape((h, w))

    # Case B: shape like (N,1)
    if frame.ndim == 2 and frame.shape[1] == 1 and frame.shape[0] > 1_000_000:
        n = frame.shape[0]
        if n == 921600:
            return frame.reshape((720, 1280))
        elif n == 1843200:
            return frame.reshape((720, 2560))
        elif n == 2073600:
            return frame.reshape((1080, 1920))
        else:
            h = 720
            w = n // h
            return frame.reshape((h, w))

    # Already 2-D in a sane way, just return
    return frame

def rand_telegraph(length: int) -> np.ndarray:
    return np.where(np.random.rand(length) < 0.5, -1, +1).astype(np.int8)

def build_stim_sequence(low_val, high_val, hold_frames, run_seconds, est_fps, mode="rand_telegraph"):
    """
    Returns:
      u_seq      : np.array of DAC levels per frame (len N)
      stim_bits  : np.array of +1 / -1 tags per frame, same length
    """
    total_frames_est = int(run_seconds * est_fps)
    n_bits = max(1, total_frames_est // max(1, hold_frames))

    bits = np.where(np.random.rand(n_bits) < 0.5, -1, +1).astype(np.int8)
    levels = np.where(bits > 0,
                      np.full_like(bits, high_val, dtype=np.int16),
                      np.full_like(bits,  low_val, dtype=np.int16))

    u_seq = np.repeat(levels, hold_frames)
    stim_bits = np.repeat(bits, hold_frames)

    if len(u_seq) < total_frames_est:
        pad = total_frames_est - len(u_seq)
        u_seq = np.concatenate([u_seq, np.full(pad, u_seq[-1], dtype=np.int16)])
        stim_bits = np.concatenate([stim_bits, np.full(pad, stim_bits[-1], dtype=np.int8)])

    return u_seq.astype(np.int16), stim_bits.astype(np.int8)

# ------------------------------------------------------------
# NEW: sector mean brightness helper (linear observable, 0..255)
# ------------------------------------------------------------
def sector_means(frame, rows=3, cols=2):
    """Compute mean brightness per sector on a rows×cols grid."""
    if frame.ndim == 2:
        v = frame
    else:
        v = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = v.shape
    ch, cw = h // rows, w // cols
    out = []
    for r in range(rows):
        for c in range(cols):
            y0, y1 = r * ch, (r + 1) * ch if r < rows - 1 else h
            x0, x1 = c * cw, (c + 1) * cw if c < cols - 1 else w
            tile = v[y0:y1, x0:x1]
            out.append(float(np.mean(tile)) if tile.size else float('nan'))
    return out  # len 6

# ------------------------------------------------------------
# NEW: causal lag estimate (|corr|, lags >= 0)
# ------------------------------------------------------------
def causal_lag_seconds(u_arr: np.ndarray, y_arr: np.ndarray, t_arr: np.ndarray) -> float:
    if len(t_arr) < 3 or len(u_arr) < 3 or len(y_arr) < 3:
        return 0.0
    # z-normalize
    u = (u_arr - np.mean(u_arr)) / (np.std(u_arr) + 1e-9)
    y = (y_arr - np.mean(y_arr)) / (np.std(y_arr) + 1e-9)
    corr = correlate(y, u, mode="full")
    lags = np.arange(-len(u) + 1, len(u))
    # causal mask
    mask = lags >= 0
    if not np.any(mask):
        return 0.0
    best_idx = int(np.argmax(np.abs(corr[mask])))
    best_lag_frames = int(lags[mask][best_idx])
    Ts = np.median(np.diff(t_arr)) if len(t_arr) > 1 else (1.0 / 30.0)
    return float(best_lag_frames * Ts)

# ------------------------------------------------------------
# Core data run (single light)
# ------------------------------------------------------------
def run_single_light(light_name: str, exposure_ms: float, gain: float):
    """
    Run one identification capture for a single light:
    - locks exposure (hard fail if we can't)
    - generates stimulus u_seq
    - logs per-frame data to CSV with metadata header (S11..S32 %dark + mu_Sxx brightness)
    - saves plot and prints delay estimate (causal |corr|, S11 only)
    """
    global DARK_THRESH, HIGH_VAL, LOW_VAL, RUN_SECONDS, STIM_HOLD_FRAMES

    if light_name not in PIDPolicy.CHANNEL_MAP:
        print(f"[ERR] Unknown light '{light_name}'. Skipping.")
        return

    col, ch = PIDPolicy.CHANNEL_MAP[light_name]
    print(f"[INIT] Driving light '{light_name}'  (col={col}, ch={ch})")
    print(f"[INIT] Using analysis camera index {CAM_INDEX} ({CAM_DEVICE})")
    print(f"[INIT] Target exposure={exposure_ms:.2f} ms  gain={gain}")

    # 0. Zero everything before this light's run
    zero_all_lights()
    time.sleep(getattr(C, "SWITCH_SETTLE_S", 0.05))

    # 1. Force manual exposure. NO try/except: if this fails we WANT to die.
    EC.set_exposure_manual(
        exposure_ms=exposure_ms,
        gain=gain,
        dev=CAM_DEVICE
    )
    print(f"[OV9281] Manual exposure: {exposure_ms:.2f} ms, gain={gain}")

    # 2. Open camera
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"[ERR] Could not open camera index {CAM_INDEX} ({CAM_DEVICE})")
        return

    # request raw mono so OpenCV doesn't auto-convert to RGB
    try:
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    except Exception:
        pass
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"Y800"))
    if int(cap.get(cv2.CAP_PROP_FOURCC)) == 0:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"GREY"))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

    # one test frame just to learn shape / mono-ness
    ok, test_frame = cap.read()
    if not ok or test_frame is None:
        print("[ERR] Camera failed first capture")
        cap.release()
        return

    test_frame = reshape_if_flat_ov9281(test_frame)

    h0, w0 = test_frame.shape[:2]
    is_mono = (test_frame.ndim == 2) or (test_frame.ndim == 3 and test_frame.shape[2] == 1)
    chans = 1 if is_mono else (test_frame.shape[2] if test_frame.ndim == 3 else 1)

    print(f"[INIT] Camera OK {w0}x{h0} — {'MONO' if is_mono else 'COLOR'} ({chans}ch)")

    # 2b. Instantiate the LightingAnalyzer (3×2) for %dark
    analyzer = LightingAnalyzer(threshold_value=DARK_THRESH, rows=3, cols=2)
    print(f"[INIT] Using LightingAnalyzer (3r, 2c) with thresh={DARK_THRESH}")

    # 3. Build the stimulus sequence we plan to apply
    EST_FPS = 30.0
    u_seq, stim_bits = build_stim_sequence(
        low_val=LOW_VAL,
        high_val=HIGH_VAL,
        hold_frames=STIM_HOLD_FRAMES,
        run_seconds=RUN_SECONDS,
        est_fps=EST_FPS,
        mode="rand_telegraph"
    )
    total_target_frames = len(u_seq)
    print(f"[INIT] u_seq length={total_target_frames}, preview={u_seq[:20]}")

    # 4. Prep logging
    os.makedirs("logs", exist_ok=True)
    timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = f"logs/idlog_{timestamp_str}_{light_name}_c{col}ch{ch}.csv"
    png_path = f"logs/idlog_{timestamp_str}_{light_name}_c{col}ch{ch}.png"

    csvfile = open(csv_path, "w", newline="")
    writer = csv.writer(csvfile)

    meta = {
        "timestamp": timestamp_str,
        "cam_device": CAM_DEVICE,
        "res": f"{w0}x{h0}",
        "exposure_ms": exposure_ms,
        "gain": gain,
        "stimulus": "rand_telegraph",
        "low": LOW_VAL,
        "high": HIGH_VAL,
        "hold_frames": STIM_HOLD_FRAMES,
        "test_light": light_name,
        "light_col": col,
        "light_channel": ch,
        "duration_s": RUN_SECONDS,
        "dark_thresh": DARK_THRESH,
        "notes": "door closed, lab lights on, ~1m distance"
    }
    for k, v in meta.items():
        csvfile.write(f"# {k}: {v}\n")

    # NEW CSV header: add mu_Sxx (mean brightness)
    writer.writerow([
        "frame_idx","t_s","dt_s",
        "u_dac","stim_bit",
        # linear brightness per sector
        "mu_S11","mu_S12","mu_S21","mu_S22","mu_S31","mu_S32",
        # %dark per sector (from LightingAnalyzer)
        "y_S11","y_S12","y_S21","y_S22","y_S31","y_S32",
        "frame_drop"
    ])

    # 5. Run acquisition loop
    print("[LOOP] Starting acquisition...")

    frame_times = []
    y_list = []   # S11 %dark series for plotting/correlation
    mu_list = []  # S11 mean brightness (optional visual later)
    t_start = time.perf_counter()
    frame_idx = -1

    # set initial DAC level before loop
    current_level = int(u_seq[0])
    write_test_light(light_name, current_level)

    while True:
        frame_idx += 1
        if frame_idx >= total_target_frames:
            print("[LOOP] Done / time limit.")
            break

        ok, frame = cap.read()
        if not ok or frame is None:
            print("[WARN] dropped frame from camera")
            continue

        frame = reshape_if_flat_ov9281(frame)

        now_s = time.perf_counter() - t_start
        frame_times.append(now_s)

        # dt/frame_drop
        if len(frame_times) == 1:
            dt_s = 0.0
            frame_drop_flag = 0
        else:
            dt_s = frame_times[-1] - frame_times[-2]
            if len(frame_times) > 10:
                med_dt = np.median(np.diff(frame_times[-10:]))
            else:
                med_dt = dt_s
            frame_drop_flag = 1 if dt_s > (1.5 * med_dt) else 0

        # ---- sector metrics
        try:
            _overall_dark, cell_dark, _annot = analyzer.analyze(frame)
        except Exception as e:
            print(f"[WARN] analyzer.analyze() failed: {e}")
            cell_dark = None

        if cell_dark is None or len(cell_dark) != 6:
            cell_dark = [-1.0] * 6  # ensure 6 values are logged

        # mean brightness (linear)
        mu = sector_means(frame, rows=3, cols=2)
        if mu is None or len(mu) != 6:
            mu = [float('nan')] * 6

        # Track S11 for plotting/correlation
        y_list.append(cell_dark[0])
        mu_list.append(mu[0])

        # intended command for this frame
        current_level = int(u_seq[frame_idx])
        current_bit   = int(stim_bits[frame_idx])

        # only write DAC if it changed
        if frame_idx > 0:
            prev_level = int(u_seq[frame_idx - 1])
            if current_level != prev_level:
                write_test_light(light_name, current_level)

        # log row (mu + 6 sector %dark)
        writer.writerow([
            frame_idx,
            round(now_s, 6),
            round(dt_s, 6),
            current_level,
            current_bit,
            round(float(mu[0]), 4),  # mu_S11
            round(float(mu[1]), 4),  # mu_S12
            round(float(mu[2]), 4),  # mu_S21
            round(float(mu[3]), 4),  # mu_S22
            round(float(mu[4]), 4),  # mu_S31
            round(float(mu[5]), 4),  # mu_S32
            round(float(cell_dark[0]), 4),  # y_S11
            round(float(cell_dark[1]), 4),  # y_S12
            round(float(cell_dark[2]), 4),  # y_S21
            round(float(cell_dark[3]), 4),  # y_S22
            round(float(cell_dark[4]), 4),  # y_S31
            round(float(cell_dark[5]), 4),  # y_S32
            frame_drop_flag
        ])

        # progress every ~60 frames
        if frame_idx % 60 == 0 and frame_idx != 0:
            print(f"[LOOP] frame={frame_idx}, t={now_s:.1f}s, S11_dark={cell_dark[0]:.1f}, mu_S11={mu[0]:.1f}")

    # end loop

    cap.release()
    csvfile.close()

    # 6. Correlation-based *causal* delay estimate (using S11 series)
    t_arr = np.array(frame_times, dtype=float)
    u_arr = u_seq[:len(frame_times)].astype(float)
    y_arr = np.array(y_list[:len(frame_times)], dtype=float)

    lag_s = causal_lag_seconds(u_arr, y_arr, t_arr)

    print(f"[DONE] Captured {len(t_arr)} samples")
    print(f"[DONE] Causal delay (S11) ≈ {lag_s:.3f}s")
    print(f"[DONE] CSV saved: {csv_path}")

    # 7. Plot and save (S11 %dark vs command)
    plt.figure(figsize=(8,4))
    plt.plot(t_arr, u_arr, label="u_dac (command)")
    plt.plot(t_arr, y_arr, label="y_S11 (%dark)")
    plt.xlabel("time [s]")
    plt.ylabel("signal level / %dark")
    plt.title(f"{light_name}  col={col} ch={ch}  {timestamp_str}  (causal lag ≈ {lag_s:.3f}s)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(png_path)
    plt.close()

    print(f"[DONE] Plot saved: {png_path}")

    # 8. Zero after this light's run
    zero_all_lights()
    time.sleep(getattr(C, "SWITCH_SETTLE_S", 0.05))

# ------------------------------------------------------------
# NEW: run all lights sequentially (data collection only)
# ------------------------------------------------------------
def get_all_lights_sorted():
    # sort by (column, channel, name) so logs group physically
    items = []
    for name, (col, ch) in PIDPolicy.CHANNEL_MAP.items():
        items.append((col, ch, name))
    items.sort()
    return [name for (_, _, name) in items]

def run_all_lights(exposure_ms: float, gain: float):
    names = get_all_lights_sorted()
    print("\n[RUN-ALL] Sequence:", ", ".join(names))
    for i, ln in enumerate(names, 1):
        print(f"\n[RUN-ALL] {i}/{len(names)}  -> {ln}")
        try:
            run_single_light(ln, exposure_ms, gain)
        except KeyboardInterrupt:
            print("[RUN-ALL] Interrupted by user.")
            break
        except Exception as e:
            print(f"[RUN-ALL] Error on light '{ln}': {e}")
    print("\n[RUN-ALL] Complete.")

# ------------------------------------------------------------
# Interactive menu
# ------------------------------------------------------------
def interactive_menu():
    """
    Simple TUI loop:
    - run ALL lights (sequentially)
    - run ONE light (uses TEST_LIGHT_NAME)
    - tweak test params
    - view latest plot
    - zero all lights
    - exit cleanly
    """
    global DARK_THRESH, HIGH_VAL, LOW_VAL, RUN_SECONDS, STIM_HOLD_FRAMES, TEST_LIGHT_NAME

    # These are live-tunable and get passed into runs
    exposure_ms = 8.0
    gain        = 10.0

    while True:
        print("\n============================")
        print("🔧 System ID Interactive Menu")
        print("============================")
        print(f"1) Run ALL lights          (each ~{RUN_SECONDS:.1f}s)")
        print(f"2) Run ONE light           (current = {TEST_LIGHT_NAME})")
        print(f"3) Set exposure time       (current = {exposure_ms:.2f} ms)")
        print(f"4) Set gain                (current = {gain})")
        print(f"5) Set dark pixel thresh   (current = {DARK_THRESH})")
        print(f"6) Set HIGH DAC value      (current = {HIGH_VAL})")
        print(f"7) Set LOW DAC value       (current = {LOW_VAL})")
        print(f"8) Set run duration        (current = {RUN_SECONDS:.1f}s)")
        print("9) Show last plot")
        print("A) Zero all lights")
        print("0) Exit")
        choice = input("Select option: ").strip().upper()

        if choice == "1":
            print("\n▶ Running ALL lights...\n")
            try:
                # lock manual exposure before runs so failure is immediate
                EC.set_exposure_manual(exposure_ms=exposure_ms, gain=gain, dev=CAM_DEVICE)
                print(f"[OV9281] Manual exposure pre-run: {exposure_ms:.2f} ms, gain={gain}")
            except Exception as e:
                print("[FATAL] Could not set manual exposure BEFORE run.")
                print("       You are NOT collecting valid data.")
                print(f"       Error from ExposureControl: {e}")
                continue
            run_all_lights(exposure_ms, gain)

        elif choice == "2":
            print("\n▶ Running ONE light...\n")
            # keep the quick single-light path for ad-hoc checks
            try:
                EC.set_exposure_manual(exposure_ms=exposure_ms, gain=gain, dev=CAM_DEVICE)
                print(f"[OV9281] Manual exposure pre-run: {exposure_ms:.2f} ms, gain={gain}")
            except Exception as e:
                print("[FATAL] Could not set manual exposure BEFORE run.")
                print("       You are NOT collecting valid data.")
                print(f"       Error from ExposureControl: {e}")
                continue
            # allow on-the-fly selection
            name_in = input(f"Enter light name (blank = {TEST_LIGHT_NAME}): ").strip()
            if name_in:
                TEST_LIGHT_NAME = name_in
            run_single_light(TEST_LIGHT_NAME, exposure_ms, gain)

        elif choice == "3":
            try:
                exposure_ms = float(input("Enter exposure time (ms): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "4":
            try:
                gain = float(input("Enter gain value: ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "5":
            try:
                DARK_THRESH = int(input("Enter dark pixel threshold (0–255): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "6":
            try:
                HIGH_VAL = int(input("Enter HIGH DAC value (0–255): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "7":
            try:
                LOW_VAL = int(input("Enter LOW DAC value (0–255): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "8":
            try:
                RUN_SECONDS = float(input("Enter run duration (s): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "9":
            latest = sorted(glob.glob("logs/idlog_*.png"))
            if not latest:
                print("No plots found.")
            else:
                last_plot = latest[-1]
                print(f"Opening {last_plot}...")
                try:
                    subprocess.run(["xdg-open", last_plot])
                except Exception as e:
                    print(f"Could not open image viewer: {e}")

        elif choice == "A":
            zero_all_lights()

        elif choice == "0":
            print("Zeroing all lights before exit...")
            zero_all_lights()
            print("Goodbye!")
            break

        else:
            print("Invalid choice.")

# ------------------------------------------------------------
# SINGLE entry point
# ------------------------------------------------------------
if __name__ == "__main__":
    interactive_menu()
