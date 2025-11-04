#!/usr/bin/env python3
"""
Program 2: Data Aggregation & Prep Script (Python/Pandas)

Purpose
-------
- Load PSBR experiment CSV logs.
- Parse metadata (channel under test) from filename/header comments.
- Build aligned dataset with u_k (10 DAC channels), y_k (6 sector outputs), y_{k+1}.
- Perform basic QA: NaNs, sampling time stats, optional plots.
- Save to training_data.pkl (and .csv) for identification.

Usage
-----
python mpc_data_prep.py --glob "/path/to/idlog_*.csv" --outdir "./build" --plots

Notes
-----
- Assumes per-log CSV with leading '#' metadata lines and header row like:
  frame_idx,t_s,dt_s,u_dac,stim_bit,mu_S11,...,y_S32,frame_drop
- File name contains "..._c{col}ch{idx}.csv" where col in {1,2}, idx in {1..5}.
  Channel index mapping: ch_idx = (col-1)*5 + idx  in 1..10
"""
import argparse, re, json
from pathlib import Path
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

U_COLS = [f"u_dac_{i}" for i in range(1, 11)]
Y_COLS = [f"y_S{r}{c}" for r in (1,2,3) for c in (1,2)]  # y_S11..y_S32

def parse_header_comment_block(path: Path) -> dict:
    meta = {}
    with path.open("r") as f:
        for line in f:
            if not line.startswith("#"):  # header comments done
                break
            line = line[1:].strip()
            if ":" in line:
                k,v = line.split(":",1)
                meta[k.strip()] = v.strip()
    return meta

def channel_index_from_filename(path: Path) -> int:
    # expects suffix like ..._c{col}ch{idx}.csv
    m = re.search(r"_c(\d)ch(\d)\.csv$", path.name)
    if not m:
        return None
    col = int(m.group(1))
    ch = int(m.group(2))
    return (col-1)*5 + ch  # 1..10

def load_one(path: Path) -> pd.DataFrame:
    meta = parse_header_comment_block(path)
    df = pd.read_csv(path, comment="#")
    # normalize column names (strip spaces)
    df.columns = [c.strip() for c in df.columns]
    # basic required columns
    required = {"frame_idx","t_s","dt_s","u_dac","stim_bit"} | set(Y_COLS)
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise ValueError(f"{path.name}: missing columns {missing}")
    # Build the 10-DAC vector
    u = pd.DataFrame(0, index=df.index, columns=U_COLS, dtype=float)
    ch_idx = channel_index_from_filename(path)
    if ch_idx is None:
        # fallback to metadata 'light_col' / 'light_channel'
        try:
            col = int(meta.get("light_col","1"))
            ch = int(meta.get("light_channel","1"))
            ch_idx = (col-1)*5 + ch
        except Exception:
            ch_idx = 1
    u_col = f"u_dac_{ch_idx}"
    u[u_col] = df["u_dac"].astype(float)
    # Build base frame
    base = df[["frame_idx","t_s","dt_s","u_dac","stim_bit"] + Y_COLS].copy()
    base["series_id"] = path.stem  # to respect sequence boundaries
    # concat u columns
    out = pd.concat([base, u], axis=1)
    # alignment: define y_{k+1}
    for y in Y_COLS:
        out[f"{y}_plus1"] = out[y].shift(-1)
    # Drop last row per series (NaN in y_{k+1})
    out = out.iloc[:-1, :]
    # Keep metadata columns for reference
    for mk, mv in meta.items():
        meta_key = f"meta_{mk.replace(' ','_')}"
        out[meta_key] = mv
    return out

def summarize_sampling(df: pd.DataFrame):
    dt = df["dt_s"]
    print(f"Sampling time (sec): Mean={dt.mean():.6f}, Std={dt.std():.6f}, Min={dt.min():.6f}, Max={dt.max():.6f}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--glob", default="/mnt/data/idlog_*.csv", help="Glob for input CSVs")
    ap.add_argument("--outdir", default="./build", help="Output directory")
    ap.add_argument("--plots", action="store_true", help="Save plots for inputs/outputs")
    args = ap.parse_args()

    paths = sorted(Path().glob(args.glob)) if not str(args.glob).startswith("/") else sorted(Path("/").glob(args.glob[1:]))
    if len(paths) == 0:
        # secondary attempt: use direct glob with Path
        paths = sorted(Path().glob(args.glob))

    if not paths:
        raise SystemExit(f"No files matched: {args.glob}")

    frames = []
    for p in paths:
        try:
            frames.append(load_one(Path(p)))
        except Exception as e:
            print(f"WARNING: skipping {p}: {e}")

    df = pd.concat(frames, axis=0, ignore_index=True)

    # QA: NaNs
    nan_counts = df.isna().sum()
    print("NaN counts:\n", nan_counts[nan_counts>0].sort_values(ascending=False))

    # Enforce dropna on any remaining NaNs
    n_before = len(df)
    df = df.dropna().reset_index(drop=True)
    print(f"Dropped {n_before - len(df)} rows due to NaNs after alignment. Final rows: {len(df)}")

    # Sampling stats
    summarize_sampling(df)

    # Build the learning table: u_k, y_k, y_{k+1}
    u_cols = U_COLS
    y_cols = Y_COLS
    y_plus_cols = [f"{y}_plus1" for y in y_cols]

    learning = df[["t_s","dt_s","series_id"] + u_cols + y_cols + y_plus_cols].copy()

    # Save outputs
    outdir = Path(args.outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    pkl_path = outdir / "training_data.pkl"
    csv_path = outdir / "training_data.csv"

    learning.to_pickle(pkl_path)
    learning.to_csv(csv_path, index=False)
    print(f"Wrote {pkl_path} and {csv_path}")

    # Optional plots
    if args.plots:
        # Note: no specific colors; one figure per group
        fig1 = plt.figure()
        for c in u_cols:
            plt.plot(learning["t_s"], learning[c], label=c, linewidth=1.0)
        plt.legend(ncol=2, fontsize=8)
        plt.xlabel("t_s (s)")
        plt.ylabel("u_dac (code)")
        plt.title("All 10 input channels over time (concatenated)")
        fig1.tight_layout()
        fig1.savefig(outdir / "inputs_over_time.png", dpi=160)
        plt.close(fig1)

        fig2 = plt.figure()
        for c in y_cols:
            plt.plot(learning["t_s"], learning[c], label=c, linewidth=1.0)
        plt.legend(ncol=3, fontsize=8)
        plt.xlabel("t_s (s)")
        plt.ylabel("darkness ratio (%)")
        plt.title("All 6 outputs over time (concatenated)")
        fig2.tight_layout()
        fig2.savefig(outdir / "outputs_over_time.png", dpi=160)
        plt.close(fig2)

if __name__ == "__main__":
    main()
