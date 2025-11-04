
#!/usr/bin/env python3
"""
mpc_controller_fast.py — Parameterized small-horizon QP MPC for lighting control.

Loads A, B from mpc_model.npz and builds a single cvxpy Problem with Parameters.
At runtime, update (y0, u_prev, r_traj[, p_coeffs]) and solve. Designed for Pi 5.

Key features:
  - Parameterized cvxpy problem (no rebuild per frame)
  - Warm-started OSQP solve
  - Slew and magnitude constraints (anti-flicker + safety)
  - Optional linear power budget p^T u <= P_max
  - Heartbeat logging hook

Note: Requires cvxpy (`pip install cvxpy`) and OSQP solver (`pip install osqp`).
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

try:
    import cvxpy as cp
except Exception as e:
    cp = None


@dataclass
class MPCConfig:
    horizon: int = 8              # small horizon for speed
    du_max: float = 12.0          # codes/frame slew
    u_min: float = 0.0
    u_max: float = 255.0
    Q_diag: Optional[np.ndarray] = None  # length ny
    R_diag: Optional[np.ndarray] = None  # length nu
    lam: float = 0.0              # optional L1 usage penalty
    P_max: Optional[float] = None # total power budget (e.g., Amps)
    p_coeffs: Optional[np.ndarray] = None # Amps/code per channel (nu,)


class FastMPC:
    def __init__(self, model_npz: str, cfg: MPCConfig):
        if cp is None:
            raise RuntimeError("cvxpy is not available. Install cvxpy to use FastMPC.")

        d = np.load(model_npz, allow_pickle=True)
        self.A: np.ndarray = d["A"]
        self.B: np.ndarray = d["B"]
        self.Ts: float = float(d["Ts"])
        self.y_cols = [str(s) for s in d["y_cols"]]
        self.u_cols = [str(s) for s in d["u_cols"]]

        ny, nu = self.B.shape
        assert self.A.shape == (ny, ny), f"A shape {self.A.shape} != {(ny, ny)}"
        self.ny, self.nu, self.N = ny, nu, int(cfg.horizon)

        # Weights
        Q_diag = cfg.Q_diag if cfg.Q_diag is not None else np.ones(ny, dtype=float)
        R_diag = cfg.R_diag if cfg.R_diag is not None else 0.05*np.ones(nu, dtype=float)
        self.Q_diag = np.asarray(Q_diag, dtype=float).copy()
        self.R_diag = np.asarray(R_diag, dtype=float).copy()
        self.lam = float(cfg.lam)

        # Constraints
        self.u_min = float(cfg.u_min); self.u_max = float(cfg.u_max); self.du_max = float(cfg.du_max)
        self.P_max = float(cfg.P_max) if cfg.P_max is not None else None
        self.p_coeffs = np.asarray(cfg.p_coeffs, dtype=float) if cfg.p_coeffs is not None else None

        # Build problem once
        self._build_problem()

        # Fallback
        self._last_good = np.zeros(nu)

    def _build_problem(self):
        ny, nu, N = self.ny, self.nu, self.N
        A, B = self.A, self.B

        # Parameters
        self.y0 = cp.Parameter(ny)
        self.u_prev = cp.Parameter(nu)
        self.r_traj = cp.Parameter((ny, N))

        # Decision variables
        self.U = cp.Variable((nu, N))
        self.Y = cp.Variable((ny, N+1))

        # Weights
        Q = cp.diag(self.Q_diag)
        R = cp.diag(self.R_diag)

        cost = 0
        constr = [self.Y[:, 0] == self.y0]
        for k in range(N):
            # Dynamics (output-as-state model)
            constr += [self.Y[:, k+1] == A @ self.Y[:, k] + B @ self.U[:, k]]

            # Magnitudes
            constr += [self.U[:, k] >= self.u_min, self.U[:, k] <= self.u_max]

            # Slew
            prev = self.u_prev if k == 0 else self.U[:, k-1]
            constr += [cp.abs(self.U[:, k] - prev) <= self.du_max]

            # Optional power
            if self.P_max is not None and self.p_coeffs is not None:
                constr += [self.p_coeffs @ self.U[:, k] <= self.P_max]

            # Quadratic cost: tracking + input magnitude
            cost += cp.quad_form(self.Y[:, k+1] - self.r_traj[:, k], Q)
            cost += cp.quad_form(self.U[:, k], R)
            if self.lam > 0:
                cost += self.lam * cp.sum(self.U[:, k])

        self._prob = cp.Problem(cp.Minimize(cost), constr)

    def compute(self, y0: np.ndarray, r_traj: np.ndarray, u_prev: np.ndarray) -> Tuple[np.ndarray, dict]:
        """Return next command (nu,) and a small heartbeat dict."""
        self.y0.value = np.asarray(y0, dtype=float).reshape(-1)
        self.r_traj.value = np.asarray(r_traj, dtype=float).reshape(self.ny, self.N)
        self.u_prev.value = np.asarray(u_prev, dtype=float).reshape(-1)

        try:
            self._prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)
            status = self._prob.status
            if status not in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
                u_cmd = self._last_good.copy()
            else:
                u_cmd = np.clip(np.array(self.U.value[:, 0]).reshape(-1), self.u_min, self.u_max)
                self._last_good = u_cmd.copy()
            hb = {
                "status": status,
                "cost": float(self._prob.value) if self._prob.value is not None else np.nan,
                "solve_time_s": float(getattr(self._prob.solver_stats, "solve_time", np.nan)),
            }
        except Exception as e:
            u_cmd = self._last_good.copy()
            hb = {"status": f"EXCEPTION: {e}", "cost": np.nan, "solve_time_s": np.nan}

        return u_cmd, hb


if __name__ == "__main__":
    # Tiny smoke test (random numbers) so the file runs without hardware.
    cfg = MPCConfig(horizon=6)
    try:
        mpc = FastMPC("/mnt/data/mpc_model.npz", cfg)
        ny, nu, N = mpc.ny, mpc.nu, mpc.N
        y0 = np.zeros(ny); r = np.tile(10.0*np.ones(ny), (N, 1)).T; u_prev = np.zeros(nu)
        u, hb = mpc.compute(y0, r, u_prev)
        print("u:", u, "heartbeat:", hb)
    except Exception as e:
        print("MPC init/solve skipped:", e)
