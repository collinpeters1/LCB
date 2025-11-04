
#!/usr/bin/env python3
"""
mpc_model_info.py — quick introspection of mpc_model.npz
"""
import numpy as np
from pathlib import Path

p = Path("/mnt/data/mpc_model.npz")
if not p.exists():
    print("Model file not found:", p)
else:
    d = np.load(p, allow_pickle=True)
    A, B = d["A"], d["B"]
    Ts = float(d["Ts"])
    y_cols = d["y_cols"].tolist()
    u_cols = d["u_cols"].tolist()
    print("Shapes: A", A.shape, "B", B.shape)
    print("Ts (s):", Ts)
    print("y_cols:", y_cols)
    print("u_cols:", u_cols)
