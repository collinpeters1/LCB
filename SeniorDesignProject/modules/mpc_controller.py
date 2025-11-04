
# mpc_controller.py
# High-speed, parameterized MPC for ILCS (6 outputs, 10 inputs), tuned for Raspberry Pi.
# - Vectorized "condensed" formulation for speed (no per-step for-loop during solve).
# - Problem is built ONCE; per-frame we only update Parameters and warm-start.
# - Robust to solver failure (fallback to last good command).
# - Optional power/thermal constraint (disabled by default; enable in mpc_config.py).
#
# State y_k     : 6-element vector of sector darkness [%], order [S11,S12,S21,S22,S31,S32]
# Control u_k   : 10-element vector of DAC codes [0..255], order see spi_dac.NAMES
#
# Model form: y_{k+1} = A y_k + B u_k   (E d_k omitted by default)
#
# Dependencies: numpy, cvxpy (OSQP), time

from __future__ import annotations
import numpy as np
import cvxpy as cp
import time

def _build_condensed(A: np.ndarray, B: np.ndarray, N: int):
    """Pre-compute A_bar (N*ny x ny) and B_bar (N*ny x N*nu)."""
    ny, nu = B.shape
    A_bar = np.zeros((ny*N, ny), dtype=float)
    B_bar = np.zeros((ny*N, nu*N), dtype=float)

    # Powers of A: A^1 .. A^N
    Apow = np.eye(ny)
    for i in range(N):  # i = 0..N-1 -> A^{i+1}
        Apow = Apow @ A
        A_bar[i*ny:(i+1)*ny, :] = Apow
        # Fill B_bar block row i: [A^{i}B, A^{i-1}B, ..., B]
        Aip = np.eye(ny)
        for j in range(i, -1, -1):  # j = i .. 0
            # block at column j is A^{i-j} B
            B_block = Aip @ B
            B_bar[i*ny:(i+1)*ny, j*nu:(j+1)*nu] = B_block
            Aip = Aip @ A
    return A_bar, B_bar

class MPCController:
    """
    Fast condensed MPC with slew constraints and optional power constraint.
    Build once, update parameters per frame, solve with warm_start.
    """
    def __init__(self, A: np.ndarray, B: np.ndarray, *, N: int,
                 Q_diag: np.ndarray, R_diag: np.ndarray,
                 u_min: np.ndarray, u_max: np.ndarray,
                 du_max: float,
                 lam: float = 0.0,
                 p_coeffs: np.ndarray | None = None,
                 P_max: float | None = None,
                 slack_penalty: float = 1e6):
        A = np.array(A, dtype=float)
        B = np.array(B, dtype=float)

        ny, nu = B.shape
        assert A.shape == (ny, ny), f"A must be {(ny,ny)}, got {A.shape}"
        assert Q_diag.shape == (ny,), f"Q_diag must be (ny,), got {Q_diag.shape}"
        assert R_diag.shape == (nu,), f"R_diag must be (nu,), got {R_diag.shape}"
        assert u_min.shape == (nu,) and u_max.shape == (nu,)

        # Store basics
        self.A, self.B = A, B
        self.ny, self.nu, self.N = ny, nu, N
        self.Q_diag = Q_diag.copy()
        self.R_diag = R_diag.copy()
        self.u_min = u_min.copy()
        self.u_max = u_max.copy()
        self.du_max = float(du_max)
        self.lam = float(lam)
        self.p_coeffs = None if p_coeffs is None else np.array(p_coeffs, dtype=float).reshape(-1)
        self.P_max = P_max
        self.slack_penalty = float(slack_penalty)

        # Precompute condensed matrices
        self.A_bar, self.B_bar = _build_condensed(A, B, N)

        # Variables and Parameters
        self.U_vec = cp.Variable(self.nu * self.N)                 # [u_0; u_1; ...; u_{N-1}]
        self.S_vec = cp.Variable(self.ny * self.N)                 # slack on tracking error (always-feasible)
        self.y0_param = cp.Parameter(self.ny)                      # current y_k
        self.u_prev_param = cp.Parameter(self.nu)                  # last applied u_{k-1}
        self.r_vec_param = cp.Parameter(self.ny * self.N)          # stacked refs [r_1..r_N]

        # Build objective: ||sqrt(Q) (Y - r - S)||^2 + ||sqrt(R) U||^2 + lam*1^T U + big*||S||^2
        QN = np.tile(self.Q_diag, self.N)
        RN = np.tile(self.R_diag, self.N)

        Y_vec = self.A_bar @ self.y0_param + self.B_bar @ self.U_vec
        err_vec = Y_vec - self.r_vec_param - self.S_vec

        cost = cp.sum_squares(cp.multiply(np.sqrt(QN), err_vec))
        cost += cp.sum_squares(cp.multiply(np.sqrt(RN), self.U_vec))
        if self.lam > 0.0:
            cost += self.lam * cp.sum(self.U_vec)
        cost += self.slack_penalty * cp.sum_squares(self.S_vec)

        # Constraints
        constr = []
        # Bounds 0<=u<=255 (vectorized)
        U_flat = self.U_vec
        uminN = np.tile(self.u_min, self.N)
        umaxN = np.tile(self.u_max, self.N)
        constr += [U_flat >= uminN, U_flat <= umaxN]

        # Slew constraints |u_k - u_{k-1}| <= du_max
        U_steps = cp.reshape(self.U_vec, (self.nu, self.N))
        constr += [cp.abs(U_steps[:, 0] - self.u_prev_param) <= self.du_max]
        if self.N > 1:
            constr += [cp.abs(cp.diff(U_steps, axis=1)) <= self.du_max]

        # Optional power constraint p^T u_k <= P_max (applied at each k)
        if (self.p_coeffs is not None) and (self.P_max is not None):
            p = self.p_coeffs.reshape(1, -1)  # (1,nu)
            for k in range(self.N):
                constr += [p @ U_steps[:, k] <= self.P_max]

        # Cache the problem
        self.problem = cp.Problem(cp.Minimize(cost), constr)
        self.last_good_u = np.zeros(self.nu, dtype=float)

    def compute(self, y0: np.ndarray, r_traj: np.ndarray, u_prev: np.ndarray,
                *, solver="OSQP", max_iter=4000, eps_abs=1e-3, eps_rel=1e-3):
        """
        One MPC step.
        r_traj: shape (ny,N) reference (columns are future steps).
        Returns (u_cmd (nu,), status, solve_time_ms, cost)
        """
        y0 = np.array(y0, dtype=float).reshape(self.ny)
        r_traj = np.array(r_traj, dtype=float).reshape(self.ny, self.N)
        u_prev = np.array(u_prev, dtype=float).reshape(self.nu)

        # Update parameters
        self.y0_param.value = y0
        self.u_prev_param.value = u_prev
        self.r_vec_param.value = r_traj.T.flatten()

        u_cmd = self.last_good_u.copy()
        status = "not_solved"
        cost_val = None
        try:
            t0 = time.perf_counter()
            self.problem.solve(solver=getattr(cp, solver),
                               warm_start=True,
                               max_iter=max_iter,
                               eps_abs=eps_abs, eps_rel=eps_rel,
                               verbose=False)
            t_ms = (time.perf_counter() - t0) * 1000.0
            status = self.problem.status
            if status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE):
                U_all = np.array(self.U_vec.value, dtype=float).reshape(self.nu, self.N)
                u_cmd = np.clip(U_all[:, 0], self.u_min, self.u_max)
                self.last_good_u = u_cmd.copy()
                cost_val = float(self.problem.value)
            else:
                # Fallback: hold
                t_ms = (time.perf_counter() - t0) * 1000.0
            return u_cmd, status, t_ms, cost_val
        except Exception as e:
            # Fallback: hold
            return self.last_good_u.copy(), f"exception:{e}", 0.0, None
