# modules/Config.py
# Centralized constants used by main and modules. Values match your current behavior.

# GPIO pins & debounce
AUTO_PIN = 22
DN_PIN = 23
DN_DEBOUNCE_MS = 75

# Analysis camera / processing
ANALYSIS_CAMERA_INDEX = 0
THRESHOLD_DARK = 30
STEP = 3

# HDR labels (purely text labels used throughout UI)
HDR_DAY_NAME = "IMX708"
HDR_NIGHT_NAME = "IMX327"

# UI / windows
WINDOW_ANALYSIS = "Analysis (C270)"
WINDOW_HDR = "HDR Feed (CSI)"

# Timing
# original value = 0.05
LOOP_SLEEP_S = 0.05
SWITCH_SETTLE_S = 0.05

# Keys
EXIT_KEYS = ("q", "ENTER")  # informational
