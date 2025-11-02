#!/usr/bin/env python3
"""
SystemID_main.py
Fast, mono-camera system-identification test for ICLS lighting.

- Uses Config.ANALYSIS_CAMERA_INDEX to open the correct /dev/videoX
- Forces mono (Y8) capture for the OV9281 analysis cam
- Generates a random-telegraph stimulus (high/low DAC)
- Logs rich per-frame data with metadata header
- Estimates delay via correlation
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

# ------------------------------------------------------------
# CONFIG (globals the menu can tweak)
# ------------------------------------------------------------
RUN_SECONDS = 12.0           # default test duration
STIM_HOLD_FRAMES = 3         # how many frames each bit is held
HIGH_VAL = 200               # DAC high level
LOW_VAL  = 0                 # DAC low level
DARK_THRESH = 20             # pixel < thresh counted as "dark"
TEST_LIGHT_NAME = "HS22"     # which light/channel we drive
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
# DAC helper
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
# Frame reshape helpers, stats, stimulus, delay
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

def make_metric_fn(is_mono: bool):
    """
    Returns a function metric_from_frame(frame) -> %dark
    """
    def _reshape_if_flat(frame):
        if frame.ndim == 2 and frame.shape[0] == 1 and frame.shape[1] > 1e6:
            n = frame.shape[1]
            if n == 921600:
                frame = frame.reshape((720, 1280))
            elif n == 1843200:
                frame = frame.reshape((720, 2560))
            elif n == 2073600:
                frame = frame.reshape((1080, 1920))
            else:
                frame = frame.reshape((720, n // 720))
        return frame

    if is_mono:
        def metric_from_frame(frame, thresh=DARK_THRESH):
            # Fix flattened mono case
            if frame.ndim == 2 and frame.shape[1] == 1 and frame.shape[0] in (1843200, 921600):
                w = 2560 if frame.shape[0] == 1843200 else 1280
                h = 720
                frame = frame.reshape((h, w))
            else:
                frame = _reshape_if_flat(frame)

            small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)

            adaptive_thresh = max(5, min(250, int(np.mean(small) * 0.8)))
            _, mask = cv2.threshold(small, adaptive_thresh, 255, cv2.THRESH_BINARY_INV)

            return 100.0 * (np.count_nonzero(mask) / mask.size)
    else:
        def metric_from_frame(frame, thresh=DARK_THRESH):
            small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
            gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
            adaptive_thresh = max(5, min(250, int(np.mean(gray) * 0.8)))
            _, mask = cv2.threshold(gray, adaptive_thresh, 255, cv2.THRESH_BINARY_INV)
            return 100.0 * (np.count_nonzero(mask) / mask.size)

    return metric_from_frame

def roi_stats_mono(small_u8):
    """
    small_u8: 2-D uint8 image after resize.
    Returns:
      (y_metric, y_mean, y_std, y_p10, y_p90, sat_lo%, sat_hi%)
    """
    v = small_u8.reshape(-1)

    dark_mask = (small_u8 < DARK_THRESH)
    y_metric = 100.0 * (np.count_nonzero(dark_mask) / dark_mask.size)

    y_mean = float(np.mean(v))
    y_std  = float(np.std(v))
    y_p10  = float(np.percentile(v, 10))
    y_p90  = float(np.percentile(v, 90))

    sat_lo = 100.0 * (np.count_nonzero(v == 0)   / v.size)
    sat_hi = 100.0 * (np.count_nonzero(v == 255) / v.size)

    return (y_metric, y_mean, y_std, y_p10, y_p90, sat_lo, sat_hi)

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
# Core data run
# ------------------------------------------------------------
def main(exposure_ms, gain):
    """
    Run one identification capture:
    - locks exposure (hard fail if we can't)
    - generates stimulus u_seq
    - logs rich per-frame data to CSV with metadata header
    - saves plot and prints delay estimate
    """

    global DARK_THRESH, HIGH_VAL, LOW_VAL, RUN_SECONDS, STIM_HOLD_FRAMES

    print(f"[INIT] Driving light '{TEST_LIGHT_NAME}'")
    print(f"[INIT] Using analysis camera index {CAM_INDEX} ({CAM_DEVICE})")
    print(f"[INIT] Target exposure={exposure_ms:.2f} ms  gain={gain}")

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
    is_mono = (test_frame.ndim == 2) or (
        test_frame.ndim == 3 and test_frame.shape[2] == 1
    )
    chans = 1 if is_mono else (test_frame.shape[2] if test_frame.ndim == 3 else 1)

    print(f"[INIT] Camera OK {w0}x{h0} — {'MONO' if is_mono else 'COLOR'} ({chans}ch)")

    metric_from_frame = make_metric_fn(is_mono)

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
    csv_path = f"logs/idlog_{timestamp_str}.csv"
    png_path = f"logs/idlog_{timestamp_str}.png"

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
        "test_light": TEST_LIGHT_NAME,
        "duration_s": RUN_SECONDS,
        "dark_thresh": DARK_THRESH,
        "notes": "door closed, lab lights on, ~1m distance"
    }
    for k, v in meta.items():
        csvfile.write(f"# {k}: {v}\n")

    writer.writerow([
        "frame_idx","t_s","dt_s",
        "u_dac","stim_bit",
        "y_metric","y_mean","y_std",
        "y_p10","y_p90",
        "sat_lo_pct","sat_hi_pct",
        "frame_drop"
    ])

    # 5. Run acquisition loop
    print("[LOOP] Starting acquisition...")

    frame_times = []
    y_list = []
    t_start = time.perf_counter()
    frame_idx = -1

    # set initial DAC level before loop
    current_level = int(u_seq[0])
    write_test_light(TEST_LIGHT_NAME, current_level)

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

        # downsample for stats
        if is_mono:
            gray_small = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
        else:
            small_color = cv2.resize(frame, (160, 90), interpolation=cv2.INTER_AREA)
            gray_small  = cv2.cvtColor(small_color, cv2.COLOR_BGR2GRAY)

        (y_metric,
         y_mean,
         y_std,
         y_p10,
         y_p90,
         sat_lo,
         sat_hi) = roi_stats_mono(gray_small)
        y_list.append(y_metric)

        # intended command for this frame
        current_level = int(u_seq[frame_idx])
        current_bit   = int(stim_bits[frame_idx])

        # only write DAC if it changed
        if frame_idx > 0:
            prev_level = int(u_seq[frame_idx - 1])
            if current_level != prev_level:
                write_test_light(TEST_LIGHT_NAME, current_level)

        # log row
        writer.writerow([
            frame_idx,
            round(now_s, 6),
            round(dt_s, 6),
            current_level,
            current_bit,
            round(y_metric, 4),
            round(y_mean,   3),
            round(y_std,    3),
            round(y_p10,    3),
            round(y_p90,    3),
            round(sat_lo,   3),
            round(sat_hi,   3),
            frame_drop_flag
        ])

        # progress every ~60 frames
        if frame_idx % 60 == 0 and frame_idx != 0:
            print(f"[LOOP] frame={frame_idx}, t={now_s:.1f}s, y={y_metric:.1f}")

    # end loop

    cap.release()
    csvfile.close()

    # 6. Correlation-based delay estimate
    t_arr = np.array(frame_times, dtype=float)
    u_arr = u_seq[:len(frame_times)].astype(float)
    y_arr = np.array(y_list[:len(frame_times)], dtype=float)

    lag_s = 0.0
    if len(t_arr) > 3:
        def z(v):
            v = np.array(v, dtype=float)
            v = v - np.mean(v)
            s = np.std(v) + 1e-9
            return v / s
        u_n = z(u_arr)
        y_n = z(y_arr)

        corr = correlate(y_n, u_n, mode="full")
        lags = np.arange(-len(u_n)+1, len(u_n))
        best_idx = int(np.argmax(corr))
        best_lag_frames = lags[best_idx]

        if len(t_arr) > 1:
            Ts = np.median(np.diff(t_arr))
        else:
            Ts = 1.0 / 30.0
        lag_s = best_lag_frames * Ts

    print(f"[DONE] Captured {len(t_arr)} samples")
    print(f"[DONE] Delay ≈ {lag_s:.3f}s")
    print(f"[DONE] CSV saved: {csv_path}")

    # 7. Plot and save
    plt.figure(figsize=(8,4))
    plt.plot(t_arr, u_arr, label="u_dac (command)")
    plt.plot(t_arr, y_arr, label="%dark (y_metric)")
    plt.xlabel("time [s]")
    plt.ylabel("signal level / %dark")
    plt.title(f"SystemID run {timestamp_str}  (lag ≈ {lag_s:.3f}s)")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.savefig(png_path)
    plt.close()

    print(f"[DONE] Plot saved: {png_path}")

# ------------------------------------------------------------
# Interactive menu (only caller of main())
# ------------------------------------------------------------
def interactive_menu():
    """
    Simple TUI loop:
    - tweak test params
    - run a capture
    - view latest plot
    - zero all lights
    - exit cleanly
    """
    global DARK_THRESH, HIGH_VAL, LOW_VAL, RUN_SECONDS, STIM_HOLD_FRAMES

    # These are live-tunable and get passed into main()
    exposure_ms = 8.0
    gain        = 10.0

    while True:
        print("\n============================")
        print("🔧 System ID Interactive Menu")
        print("============================")
        print(f"1) Run test                (current run time = {RUN_SECONDS:.1f}s)")
        print(f"2) Set exposure time       (current = {exposure_ms:.2f} ms)")
        print(f"3) Set gain                (current = {gain})")
        print(f"4) Set dark pixel thresh   (current = {DARK_THRESH})")
        print(f"5) Set HIGH DAC value      (current = {HIGH_VAL})")
        print(f"6) Set LOW DAC value       (current = {LOW_VAL})")
        print(f"7) Set run duration        (current = {RUN_SECONDS:.1f}s)")
        print("8) Show last plot")
        print("9) Zero all lights")
        print("0) Exit")
        choice = input("Select option: ").strip()

        if choice == "1":
            print("\n▶ Running test...\n")

            # try to lock manual exposure *again* right before run so you see crash ASAP
            # this call is duplicated in main() (which will also crash if it fails),
            # but seeing it here too is nice feedback.
            try:
                EC.set_exposure_manual(
                    exposure_ms=exposure_ms,
                    gain=gain,
                    dev=CAM_DEVICE
                )
                print(f"[OV9281] Manual exposure pre-run: {exposure_ms:.2f} ms, gain={gain}")
            except Exception as e:
                print("[FATAL] Could not set manual exposure BEFORE run.")
                print("       You are NOT collecting valid data.")
                print(f"       Error from ExposureControl: {e}")
                # don't run the test at all if this fails
                continue

            # run one test using current local exposure_ms/gain
            main(exposure_ms, gain)

        elif choice == "2":
            try:
                exposure_ms = float(input("Enter exposure time (ms): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "3":
            try:
                gain = float(input("Enter gain value: ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "4":
            try:
                DARK_THRESH = int(input("Enter dark pixel threshold (0–255): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "5":
            try:
                HIGH_VAL = int(input("Enter HIGH DAC value (0–255): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "6":
            try:
                LOW_VAL = int(input("Enter LOW DAC value (0–255): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "7":
            try:
                RUN_SECONDS = float(input("Enter run duration (s): ").strip())
            except ValueError:
                print("Invalid number.")

        elif choice == "8":
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

        elif choice == "9":
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
