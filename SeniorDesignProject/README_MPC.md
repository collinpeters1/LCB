
# MPC Identification Pipeline (Pi-friendly)

This bundle provides:
- **Program 2**: `mpc_data_prep.py` — aggregate PSBR logs into an aligned training set.
- **Program 3**: `mpc_identification.py` — estimate a linear time-invariant (LTI) model `y_{k+1} = A y_k + B u_k`.

> Program 1 is your existing PSBR logger that produced the `idlog_*.csv` files.

## Quick start (on your Pi or laptop)

```bash
# 1) Build the dataset
python3 mpc_data_prep.py --glob "/mnt/data/idlog_*.csv" --outdir "/mnt/data/build" --plots

# 2) Identify the model
python3 mpc_identification.py --data "/mnt/data/build/training_data.pkl" --outdir "/mnt/data/build"
```

Artifacts written to `/mnt/data/build`:
- `training_data.pkl` and `training_data.csv`
- `inputs_over_time.png`, `outputs_over_time.png` (optional with `--plots`)
- `mpc_model.npz` (A, B, Ts, column names)
- `A_matrix.csv`, `B_matrix.csv`
- `ident_report.json` with RMSE metrics

## Column conventions

- Inputs **u**: 10 channels `u_dac_1..u_dac_10`. Each PSBR log energizes one channel;
  the script maps the active channel from the filename suffix `_c{col}ch{idx}.csv`:
  `channel_id = (col-1)*5 + idx`.
- Outputs **y**: 6 darkness ratios `y_S11..y_S32`.
- Time: `t_s` (seconds), `dt_s` (frame-to-frame).

## Model details

- Fits `[A (6x6), B (6x10)]` by least squares.
- Assumes `C = I_6`, `D = 0` for MPC.
- Uses **median dt_s** as sample time `Ts` in the saved model.

## Validation

- One-step prediction (teacher-forced) and free roll-out plots.
- Sector RMSE reported for both.

## Next steps

- Use `mpc_model.npz` inside your controller to compute receding-horizon commands
  (e.g., with cvxpy or qpOASES on the Pi).

