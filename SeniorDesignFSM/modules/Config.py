#!/usr/bin/env python3
"""
Config.py — Centralized configuration for ICLS (Pi 5)
(Updated: Night HDR cam label set to IMX290; analysis index = 0)
"""

# ============================= Pins & switches ============================= #
AUTO_PIN = 22
DN_PIN = 23
MANUAL_ACTIVE_LOW = True
DAY_HIGH_IS_DAY   = True
DN_DEBOUNCE_MS = 50

# ============================== Cameras / HDR ============================== #
HDR_DAY_NAME   = "IMX708"
HDR_NIGHT_NAME = "IMX290"   # <-- your logs show imx290 on the Night slot

# Map camera name -> Picamera2 index (as used by DualPicam.PicamFeed)
# Adjust to match how libcamera enumerated them on your Pi.
CAM_NAME_TO_NUM = {
    "IMX708": 0,
    "IMX290": 1,   # <-- updated to match the imx290 registration in your logs
}

# Optional per-camera params for PicamFeed
CAM_PARAMS = {
    "IMX708": {"af": True},
    "IMX290": {"af": False},
}

SWITCH_SETTLE_S = 0.20

# Analysis camera (OpenCV index). Most USB cams are /dev/video0.
ANALYSIS_CAMERA_INDEX = 0

# EMA smoothing
EMA_ALPHA = 0.70

# ============================== Loop & tuning ============================== #
LOOP_PERIOD_S = 0.05
DARK_T_ON = 12.0
DARK_T_OFF = 8.0
DARK_T_HOLD_MARGIN = 1.0
THRESHOLD_DARK = 10.0

RAMP_UP_CPS = 240
RAMP_DOWN_CPS = 120

FSM_TRIM_ENABLE = True
FSM_TRIM_DB = 1.5
FSM_TRIM_KP = 4.0
FSM_TRIM_RATE = 40
FSM_DEBUG = False

# ================================ DAC / SPI ================================ #
DAC_MAX = 255
SPI_BUS = 1
COL_TO_DEVICE = {1: 1, 2: 0}

LIGHT_NAMES = [
    "HS11","HS21","LS1","LF1","HF1",
    "HS12","HS22","LS2","LF2","HF2",
]

CHANNEL_MAP = {
    "HS11": (1,1), "HS21": (1,2), "LS1": (1,3), "HF1": (1,4), "LF1": (1,5),
    "HS12": (2,1), "HS22": (2,2), "LS2": (2,3), "HF2": (2,4), "LF2": (2,5),
}

DAC_MAX_SPEED_HZ = 1_000_000
DAC_MODE = 0
DAC_BITS = 8

# ============================ Overlay / UI opts ============================ #
OVERLAY_TITLE = "ICLS"
CAM_ROUTER_VERBOSE = False

# ======================= Backward-compatibility shims ====================== #
LOOP_SLEEP_S = LOOP_PERIOD_S

# ================================ Sanity ================================== #
try:
    assert isinstance(LIGHT_NAMES, list) and len(LIGHT_NAMES) == 10
    for name in LIGHT_NAMES:
        assert name in CHANNEL_MAP, f"Missing CHANNEL_MAP entry for {name}"
except AssertionError as _e:
    print(f"[Config] Warning: {_e}")
