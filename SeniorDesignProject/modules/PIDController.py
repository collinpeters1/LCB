# modules/PIDController.py
# A tiny, dependency-free controller tuned for your "hold near threshold, ramp fast when far"
# goals. It behaves like a PI controller with:
# - deadband around the setpoint to avoid oscillation,
# - proportional step scaling with error magnitude,
# - slower dimming than brightening to prevent flicker,
# - integrator clamped to keep things stable.

from dataclasses import dataclass

@dataclass
class PIDLikeConfig:
    kp: float = 0.35           # proportional gain
    ki: float = 0.1          # integral gain (small, just to "hold")
    kd: float = 0.0           # not used; kept for completeness
    deadband: float = 3.0     # ±% darkness around threshold where we "hold"
    min_step: float = 1.0     # minimum change per update when outside deadband
    max_step: float = 10.0    # maximum change per update when far from target
    down_scale: float = 0.05   # make dimming slower than brightening
    integral_limit: float = 200.0
    out_min: int = 0
    out_max: int = 255

class PIDLike:
    def __init__(self, cfg: PIDLikeConfig | None = None):
        self.cfg = cfg or PIDLikeConfig()
        self._i = 0.0
        self._prev_err = 0.0

    def reset(self):
        self._i = 0.0
        self._prev_err = 0.0

    def update(self, error: float, current: int) -> int:
        """
        error = (measured_darkness - threshold_dark)
        current = current DAC value [0..255]
        Returns new DAC value clamped to [out_min..out_max].
        """
        c = self.cfg

        # Inside the deadband: "hold" current; bleed integrator toward zero.
        if abs(error) <= c.deadband:
            self._i *= 0.9
            return int(max(c.out_min, min(c.out_max, current)))

        # Scale allowable instantaneous step with magnitude of error (0..50% -> min..max).
        mag = min(max(abs(error), 0.0), 50.0)
        step_span = c.max_step - c.min_step
        step_mag = c.min_step + (step_span * (mag / 50.0))

        # Basic PI (no D). Small integral to help "hold" near target.
        p = c.kp * error
        self._i += c.ki * error
        # Clamp the integrator
        if self._i > c.integral_limit:
            self._i = c.integral_limit
        if self._i < -c.integral_limit:
            self._i = -c.integral_limit

        delta = p + self._i

        # Limit instantaneous change to dynamic step bound.
        if delta > step_mag:
            delta = step_mag
        elif delta < -step_mag:
            delta = -step_mag

        # Make dimming (negative delta) more conservative than brightening.
        if delta < 0:
            delta *= c.down_scale

        # Compute and clamp new value.
        new_val = int(round(current + delta))
        if new_val < c.out_min:
            new_val = c.out_min
        if new_val > c.out_max:
            new_val = c.out_max

        # NEW ZERO-CROSSING RESET LOGIC:
        # If the control error crosses zero, the accumulated integral is no longer valid.
        if (error > 0 and self._prev_err < 0) or (error < 0 and self._prev_err > 0):
            self._i = 0.0

        self._prev_err = error
        return new_val
