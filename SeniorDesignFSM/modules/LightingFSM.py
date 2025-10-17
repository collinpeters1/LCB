#!/usr/bin/env python3
"""
LightingFSM.py — Per-light finite state machine for DAC control.

States:
    OFF → RAMP_UP → HOLD → RAMP_DOWN → OFF
Transitions use:
    - Hysteresis thresholds (T_on > T_off, with a small T_hold)
    - Dwell timers (min_on_s, min_off_s)
    - Debounce / re-arm timers (off_guard_s, rearm_s)
    - Asymmetric ramps (ramp_up, ramp_down)
    - Optional HOLD micro-trim (P control) with deadband and rate clamp

Usage:
    from modules.LightingFSM import LightFSM, FSMParams
    params = FSMParams()
    fsm = LightFSM(params=params, name="HS11")

    # every loop (dt in seconds)
    new_dac = fsm.step(want=desired_on["HS11"], darkness=sector_signal, dt=loop_dt)

Notes:
    - darkness is in percent (0..100), consistent with LightingAnalysis.
    - DAC values are 0..cap_on (cap_on typically 255).
    - This module is “pure control”: no SPI, no GPIO. Main pushes values via DACBus.
"""

from dataclasses import dataclass, field
from typing import Optional, Dict
import time

# ----------------------------- Tunable parameters -----------------------------

@dataclass
class FSMParams:
    # Hysteresis
    T_on: float = 12.0          # %dark to start turning on (upper threshold)
    T_off: float = 8.0          # %dark below which we consider bright enough
    hold_margin: float = 1.0    # T_hold = T_off + hold_margin

    # Ramping (DAC counts per second)
    ramp_up: int = 240          # e.g., 240 cnt/s at 20 Hz ≈ +12 / loop
    ramp_down: int = 120        # slower fade down (perceptually nicer)

    # Dwell / debounce / re-arm
    min_on_s: float = 0.6
    min_off_s: float = 0.8
    off_guard_s: float = 0.15   # time in RAMP_UP before honoring want=False
    rearm_s: float = 0.25       # time after OFF/RAMP_DOWN before re-allowing UP

    # Output limits
    cap_on: int = 255           # DAC max
    floor: int = 0              # DAC min

    # HOLD micro-trim (optional gentle P-control)
    enable_trim: bool = True
    trim_deadband: float = 1.5  # %dark; ignore tiny errors
    trim_kp: float = 4.0        # DAC counts per %dark error (small!)
    trim_rate_limit: int = 40   # max |ΔDAC| per second during trim

    # Debug flags
    debug: bool = False


# --------------------------------- FSM class ---------------------------------

@dataclass
class LightFSM:
    params: FSMParams
    name: str = "LIGHT"
    state: str = "OFF"                   # OFF, RAMP_UP, HOLD, RAMP_DOWN
    value: int = 0                       # current DAC command
    # Timestamps
    _t_last_change: float = field(default_factory=time.monotonic)
    _t_last_on: float = field(default_factory=time.monotonic)
    _t_last_off: float = field(default_factory=time.monotonic)
    # For rate limiting trim
    _trim_accum: float = 0.0

    # -------- Public API --------

    def step(self, want: bool, darkness: float, dt: float) -> int:
        """
        Advance the FSM one tick.
        want      : planner’s desire for this light (bool)
        darkness  : %dark signal this light responds to (0..100)
        dt        : loop delta time in seconds
        Returns the new integer DAC value (0..cap_on).
        """
        p = self.params
        now = time.monotonic()
        T_hold = p.T_off + p.hold_margin

        def elapsed(since: float) -> float:
            return now - since

        def go(new_state: str) -> None:
            if new_state != self.state:
                self.state = new_state
                self._t_last_change = now
                if p.debug:
                    print(f"[FSM {self.name}] → {new_state} (val={self.value})")

        # --- State logic ---
        if self.state == "OFF":
            # Allow turn-on only if planner wants it, darkness is high, and we've been off long enough.
            if want and (darkness >= p.T_on) and (elapsed(self._t_last_off) >= p.min_off_s):
                go("RAMP_UP")

        elif self.state == "RAMP_UP":
            # Increase DAC by ramp_up * dt, respect cap
            self.value = min(p.cap_on, self.value + int(p.ramp_up * max(dt, 0.0)))

            # If bright enough (darkness <= T_hold) or saturated, hold.
            if (darkness <= T_hold) or (self.value >= p.cap_on):
                self._t_last_on = now
                go("HOLD")
            # If planner flips early, allow ramp down after an off_guard delay.
            elif (not want) and (elapsed(self._t_last_change) >= p.off_guard_s):
                go("RAMP_DOWN")

        elif self.state == "HOLD":
            # Keep steady, with an optional tiny trim inside a deadband.
            if p.enable_trim:
                self._apply_trim(darkness, dt)

            # Exit HOLD if planner no longer wants the light and we've met min_on.
            if (not want) and (elapsed(self._t_last_on) >= p.min_on_s):
                go("RAMP_DOWN")
            # Optional top-up if darkness spikes and we aren't at cap.
            elif want and (darkness >= (p.T_on + 3.0)) and (self.value < p.cap_on):
                self.value = min(p.cap_on, self.value + max(1, int(0.5 * p.ramp_up * max(dt, 0.0))))

        elif self.state == "RAMP_DOWN":
            # Decrease DAC by ramp_down * dt, respect floor
            self.value = max(p.floor, self.value - int(p.ramp_down * max(dt, 0.0)))

            # If fully off, transition to OFF
            if self.value <= p.floor:
                self._t_last_off = now
                go("OFF")
            # If darkness reappears and planner wants it, allow re-arming
            elif want and (elapsed(self._t_last_off) >= p.rearm_s) and (darkness >= p.T_on):
                go("RAMP_UP")

        # Clamp and return integer DAC
        self.value = int(max(p.floor, min(p.cap_on, self.value)))
        return self.value

    def force(self, value: int, new_state: Optional[str] = None) -> None:
        """
        Hard-set the DAC (e.g., during Emergency) and optionally force a state name.
        """
        self.value = int(max(self.params.floor, min(self.params.cap_on, value)))
        if new_state:
            self.state = new_state
            self._t_last_change = time.monotonic()

    def to_dict(self) -> Dict[str, object]:
        """
        Small helper for overlays/logging.
        """
        return {"name": self.name, "state": self.state, "value": self.value}

    # -------- Internal helpers --------

    def _apply_trim(self, darkness: float, dt: float) -> None:
        """
        Micro-trim inside HOLD: very small proportional nudge toward T_off,
        with deadband and per-second rate limiting to avoid chatter.
        """
        p = self.params
        if dt <= 0.0:
            return
        # Error is how far above/below T_off we are (positive means darker than desired).
        error = darkness - p.T_off

        # Deadband: ignore tiny errors
        if abs(error) < p.trim_deadband:
            return

        # Desired change this tick before clamping
        raw_delta = p.trim_kp * error

        # Rate limit over time (counts per second)
        max_delta = p.trim_rate_limit * dt
        delta = max(-max_delta, min(max_delta, raw_delta))

        # Apply tiny nudge
        new_val = self.value + int(delta)
        self.value = int(max(p.floor, min(p.cap_on, new_val)))
