# modules/ExposureControl.py
# OV9281 exposure + simple profiles (Classroom / Lunar)
# Uses controls found on your device:
#   - auto_exposure (menu 0..3): 3 = Auto (Aperture Priority), 1 = Manual
#   - exposure_time_absolute (int, units = 100 µs)
#   - gain (0..100)

import subprocess
from modules import Config as C

EXPO_DEV = f"/dev/video{C.ANALYSIS_CAMERA_INDEX}"

def _run(args):
    subprocess.run(args, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def set_exposure_auto(dev: str = EXPO_DEV) -> bool:
    """Enable camera auto exposure (auto_exposure=3)."""
    try:
        _run(["v4l2-ctl", "-d", dev, "-c", "auto_exposure=3"])
        print("[OV9281] Auto exposure ON (auto_exposure=3)")
        return True
    except Exception as e:
        print(f"[OV9281] AUTO exposure failed: {e}")
        return False

def set_exposure_manual(exposure_ms: float = 15.0, gain: int | None = None, dev: str = EXPO_DEV) -> bool:
    """
    Manual mode + shutter time (ms). Units are 100 µs on exposure_time_absolute.
    For 60 fps, keep exposure_ms ≤ 16.7. Optionally set analog gain (0..100).
    """
    try:
        # Manual mode
        _run(["v4l2-ctl", "-d", dev, "-c", "auto_exposure=1"])
        # ms -> units of 100 µs
        units = max(1, min(int(round(exposure_ms * 10.0)), 5000))
        _run(["v4l2-ctl", "-d", dev, "-c", f"exposure_time_absolute={units}"])
        if gain is not None:
            g = max(0, min(int(gain), 100))
            _run(["v4l2-ctl", "-d", dev, "-c", f"gain={g}"])
        print(f"[OV9281] Manual exposure: {exposure_ms:.2f} ms (exposure_time_absolute={units})"
              + (f", gain={gain}" if gain is not None else ""))
        return True
    except Exception as e:
        print(f"[OV9281] MANUAL exposure failed: {e}")
        return False

# -------- Profiles --------
PROFILES = {
    # Good start for bright indoor environment
    "Classroom": {
        "exposure": "auto",       # leave camera in auto
        "exposure_ms": 15.0,      # ignored in auto
        "gain": None,             # ignored in auto
        "threshold": 30,          # analyzer threshold target
        "otsu": False,
    },
    # Stable, high-contrast environment (moon-like)
    "Lunar": {
        "exposure": "manual",     # lock exposure
        "exposure_ms": 15.0,      # ~15 ms works with 60 fps; adjust 12–16.7 as needed
        "gain": 10,               # small fixed analog gain to lift shadows, tweak 0–20
        "threshold": 70,          # higher cutoff to separate shadows
        "otsu": False,            # keep off unless you want a fallback
    },
}

PROFILE_ORDER = ["Classroom", "Lunar"]

def apply_profile(name: str, analyzer, dev: str = EXPO_DEV) -> bool:
    """Apply camera exposure + analyzer settings for the named profile."""
    p = PROFILES[name]
    ok = True
    if p["exposure"] == "auto":
        ok = set_exposure_auto(dev)
    else:
        ok = set_exposure_manual(p["exposure_ms"], p["gain"], dev)

    # Push analyzer settings if methods exist
    if hasattr(analyzer, "get_threshold") and hasattr(analyzer, "adjust_threshold"):
        cur = int(analyzer.get_threshold())
        delta = int(p["threshold"]) - cur
        if delta:
            analyzer.adjust_threshold(delta)
    if hasattr(analyzer, "set_otsu_enabled"):
        analyzer.set_otsu_enabled(bool(p["otsu"]))

    print(f"[Profile] {name}: T={p['threshold']} OTSU={'ON' if p['otsu'] else 'OFF'} "
          f"Exposure={p['exposure']}({p['exposure_ms']} ms, gain={p['gain']})")
    return ok

def cycle_profile(current_name: str, analyzer, dev: str = EXPO_DEV) -> str:
    """Cycle to the next profile and apply it. Returns the new profile name."""
    idx = PROFILE_ORDER.index(current_name) if current_name in PROFILE_ORDER else 0
    new = PROFILE_ORDER[(idx + 1) % len(PROFILE_ORDER)]
    apply_profile(new, analyzer, dev)
    return new90
