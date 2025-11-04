
# mpc_config.py
# Central spot for tuning / paths so you can adjust without editing code.

import numpy as np

# --- Paths ---
# Where the identified model is saved (npz with 'A','B'). Try local first, then /home/pi.
MODEL_PATHS = [
    "mpc_model.npz",                    # working dir
    "/home/pi/LightingControl/mpc_model.npz",
    "/home/pi/mpc_model.npz",
]

# --- Horizon & weights ---
N_HORIZON = 10     # keep small (6..12) for Pi timing
Q_TRACK = np.array([1, 1, 1, 1, 1, 1], dtype=float)     # per-sector tracking weight
R_USE   = np.array([0.05]*10, dtype=float)              # per-channel magnitude weight
LAMBDA_LINEAR = 0.0                                     # usually 0 if using R

# --- Constraints ---
U_MIN = np.zeros(10, dtype=float)
U_MAX = np.full(10, 255.0, dtype=float)
DU_MAX = 12.0                                           # codes per frame (anti-flicker)

# Power/thermal (optional). Disabled by default.
P_COEFFS = None     # e.g., np.array([0.021]*10)
P_MAX    = None     # e.g., 5.25

# --- Policy / targets ---
# Target sector darkness (%) per sector [S11,S12,S21,S22,S31,S32].
TARGET_PER_SECTOR = np.array([30.0, 30.0, 30.0, 30.0, 30.0, 30.0], dtype=float)

# --- UI / timing ---
SHOW_WINDOWS = True
SLEEP_S = 0.01

# --- Logging ---
HEARTBEAT_CSV = "mpc_heartbeat.csv"  # created next to script if non-empty string
