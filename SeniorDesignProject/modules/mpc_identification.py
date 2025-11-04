#!/usr/bin/env python3
"""
Program 3: Model Identification Script (Python/Numpy)

Model form
----------
y_{k+1} = A y_k + B u_k + e_k
State y is the 6x1 vector of darkness ratios (y_S11..y_S32), input u is 10x1 DAC codes.

Workflow
--------
- Load training_data.pkl produced by Program 2.
- Split into train (first 80%) and validation (last 20%) *by row*.
- Fit [A B] via least squares on training set.
- Validate with:
  - One-step prediction (uses actual y_k)
  - Free roll-out (uses predicted y recursively)
- Plots and metrics (RMSE) saved to outdir.
- Save mpc_model.npz containing A, B, Ts (median), and column names.
"""
import argparse, json
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

Y_COLS = [f"y_S{r}{c}" for r in (1,2,3) for c in (1,2)]
U_COLS = [f"u_dac_{i}" for i in range(1, 11)]

def fit_ab(train_df: pd.DataFrame):
    Yk = train_df[Y_COLS].to_numpy()              # N x 6
    Uk = train_df[U_COLS].to_numpy()              # N x 10
    Ykp1 = train_df[[f"{c}_plus1" for c in Y_COLS]].to_numpy()  # N x 6
    # Build regressor: Z_k = [Y_k | U_k]
    Z = np.concatenate([Yk, Uk], axis=1)          # N x 16
    # Solve Y_{k+1}^T = Theta^T Z^T via least squares
    Theta, *_ = np.linalg.lstsq(Z, Ykp1, rcond=None)  # (16 x 6)
    # Extract A (6x6) and B (10x6) but note shape; we solved Z (N x 16) * Theta (16 x 6) ~ Ykp1 (N x 6)
    A = Theta[:6, :].T   # (6 x 6)
    B = Theta[6:, :].T   # (6 x 10) desired is (6 x 10)
    # We want B to be shape (6 x 10), currently B is (6 x 10) correct.
    return A, B

def rmse(a, b):
    return float(np.sqrt(np.mean((a-b)**2)))

def validate(df: pd.DataFrame, A, B, outdir: Path):
    # One-step prediction (teacher-forced)
    Yk = df[Y_COLS].to_numpy()
    Uk = df[U_COLS].to_numpy()
    Y1 = (A @ Yk.T + B @ Uk.T).T  # N x 6
    Ykp1 = df[[f"{c}_plus1" for c in Y_COLS]].to_numpy()
    rmse_1step = {c: rmse(Y1[:,i], Ykp1[:,i]) for i,c in enumerate(Y_COLS)}

    # Free roll-out over the validation segment
    horizon = len(df)
    Yroll = np.zeros_like(Yk)
    # seed with first actual y_k
    if horizon > 0:
        Yroll[0] = Yk[0]
        for t in range(horizon-1):
            Yroll[t+1] = (A @ Yroll[t] + B @ Uk[t]).reshape(-1)

    rmse_roll = {c: rmse(Yroll[:,i], Yk[:,i]) for i,c in enumerate(Y_COLS)}

    # Plots
    # A: Compare actual y and roll-out
    fig = plt.figure(figsize=(10,8))
    rows, cols = 3, 2
    for i,c in enumerate(Y_COLS):
        ax = fig.add_subplot(rows, cols, i+1)
        ax.plot(Yk[:,i], label=f"{c} actual", linewidth=1.0)
        ax.plot(Yroll[:,i], label=f"{c} rollout", linewidth=1.0)
        ax.set_title(f"{c}")
        ax.grid(True, which="both", linestyle=":", linewidth=0.5)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(outdir / "validation_rollout.png", dpi=160)
    plt.close(fig)

    # B: One-step parity plot per channel (pred vs target)
    fig2 = plt.figure(figsize=(10,8))
    for i,c in enumerate(Y_COLS):
        ax = fig2.add_subplot(rows, cols, i+1)
        ax.plot(Ykp1[:,i], label="target", linewidth=1.0)
        ax.plot(Y1[:,i], label="1-step", linewidth=1.0)
        ax.set_title(f"{c} (one-step)")
        ax.grid(True, which="both", linestyle=":", linewidth=0.5)
        ax.legend(fontsize=8)
    fig2.tight_layout()
    fig2.savefig(outdir / "validation_onestep.png", dpi=160)
    plt.close(fig2)

    return rmse_1step, rmse_roll

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="./build/training_data.pkl", help="Path to training_data.pkl")
    ap.add_argument("--outdir", default="./build", help="Output directory for artifacts")
    args = ap.parse_args()

    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)

    df = pd.read_pickle(args.data)

    # Split
    N = len(df)
    n_train = int(0.8 * N)
    train = df.iloc[:n_train].reset_index(drop=True)
    valid = df.iloc[n_train:].reset_index(drop=True)

    # Fit
    A, B = fit_ab(train)

    # Ts (use median dt_s over entire data)
    Ts = float(df["dt_s"].median()) if "dt_s" in df.columns else None

    # Validate
    rmse_1step, rmse_roll = validate(valid, A, B, outdir)

    # Save model
    np.savez(outdir / "mpc_model.npz", A=A, B=B, Ts=Ts, y_cols=np.array(Y_COLS), u_cols=np.array(U_COLS))

    # Human-readable report
    report = {
        "rows_total": int(N),
        "rows_train": int(len(train)),
        "rows_valid": int(len(valid)),
        "Ts_median_s": Ts,
        "rmse_one_step": rmse_1step,
        "rmse_rollout": rmse_roll,
        "A_shape": A.shape,
        "B_shape": B.shape,
    }
    with open(outdir / "ident_report.json","w") as f:
        json.dump(report, f, indent=2)
    # Also dump A,B as CSV for inspection
    pd.DataFrame(A, index=Y_COLS, columns=Y_COLS).to_csv(outdir / "A_matrix.csv")
    pd.DataFrame(B, index=Y_COLS, columns=U_COLS).to_csv(outdir / "B_matrix.csv")

    # Print key info to console
    print(json.dumps(report, indent=2))
    print("Top entries of B (influence):")
    print(pd.DataFrame(B, index=Y_COLS, columns=U_COLS).round(4).to_string())

if __name__ == "__main__":
    main()
