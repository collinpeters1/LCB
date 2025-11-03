#!/usr/bin/env python3
"""
CoveragePlanner.py — Stateless-looking (internally hysteretic) selector for desired lights.

Goal:
  - Choose the minimal set of lights that covers the current dark sectors.
  - Prefer combined lights (LS/LF) when two stacked sectors are dark.
  - Use per-sector hysteresis (T_on/T_off) to avoid planner chatter.
  - Never enable a single-sector light (HS/HF) if an LS/LF already covers that sector.

Inputs:
  - sector_darkness: list[6] of %dark (0..100) in order [S11,S12,S21,S22,S31,S32]
  - thresholds: T_on (turn dark), T_off (turn bright)
  - Optional: explicit light names list, otherwise uses defaults from Config or built-in.

Outputs:
  - desired_on: dict {light_name -> bool} for all 10 lights:
      Col 1: HS11, HS21, LS1, LF1, HF1
      Col 2: HS12, HS22, LS2, LF2, HF2
  - (optionally) internal last-dark flags are updated for hysteresis.

Usage:
  from modules.CoveragePlanner import CoveragePlanner, plan_desired_on
  planner = CoveragePlanner(T_on=12.0, T_off=8.0)
  desired = planner.update(sector_darkness)

Notes:
  - Hysteresis is applied at the *sector level*:
      dark_now = True  if val >= T_on
      dark_now = False if val <= T_off
      between thresholds → keep last state
  - With three dark sectors in a column, LSx and LFx will both be True (covers all 3).
  - With just the middle sector dark, HS2x is chosen (not LS/LF).
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

# Optional import of project config (pins/names); code works without it.
try:
    from modules import Config as C
    _CFG_LIGHT_NAMES = getattr(C, "LIGHT_NAMES", None)
except Exception:
    C = None
    _CFG_LIGHT_NAMES = None


# --------------------------- Helper constants / order ---------------------------

# Sector indices in sector_darkness list
S11, S12, S21, S22, S31, S32 = 0, 1, 2, 3, 4, 5

# Canonical light order (overlays/logging read nicer in this order)
DEFAULT_LIGHT_NAMES = [
    # Column 1
    "HS11", "HS21", "LS1", "LF1", "HF1",
    # Column 2
    "HS12", "HS22", "LS2", "LF2", "HF2",
]


@dataclass
class CoveragePlanner:
    T_on: float = 12.0
    T_off: float = 8.0
    light_names: List[str] = field(default_factory=lambda: _CFG_LIGHT_NAMES or DEFAULT_LIGHT_NAMES)
    debug: bool = False

    # Internal per-sector hysteresis memory (True=dark, False=bright)
    _last_dark: List[bool] = field(default_factory=lambda: [False]*6)

    def reset(self) -> None:
        """Clear hysteresis memory (all sectors assumed bright)."""
        self._last_dark = [False]*6

    def _hysteresis_flags(self, sector_darkness: List[float]) -> List[bool]:
        """
        Apply per-sector hysteresis and return booleans for dark sectors.
        """
        out = self._last_dark[:]  # start from previous
        for i, val in enumerate(sector_darkness):
            if val >= self.T_on:
                out[i] = True
            elif val <= self.T_off:
                out[i] = False
            # else: keep previous state
        return out

    def update(self, sector_darkness: List[float]) -> Dict[str, bool]:
        """
        Compute desired_on for all 10 lights using minimal-coverage logic per column.
        """
        if len(sector_darkness) != 6:
            raise ValueError("sector_darkness must be length 6: [S11,S12,S21,S22,S31,S32]")

        # 1) Per-sector hysteresis
        dark = self._hysteresis_flags(sector_darkness)

        # 2) Initialize output dict with all lights off
        desired: Dict[str, bool] = {name: False for name in self.light_names}

        # 3) Column 1 logic (S11,S21,S31 → HS11,HS21,LS1,LF1,HF1)
        s11, s21, s31 = dark[S11], dark[S21], dark[S31]

        # Prefer combined coverage when two stacked are dark
        if s11 and s21:
            desired["LS1"] = True
        if s21 and s31:
            desired["LF1"] = True

        # Single-sector cases when not already covered by a combined light
        if s11 and not desired["LS1"]:
            desired["HS11"] = True
        if s21 and not (desired["LS1"] or desired["LF1"]):
            desired["HS21"] = True
        if s31 and not desired["LF1"]:
            desired["HF1"] = True

        # 4) Column 2 logic (S12,S22,S32 → HS12,HS22,LS2,LF2,HF2)
        s12, s22, s32 = dark[S12], dark[S22], dark[S32]

        if s12 and s22:
            desired["LS2"] = True
        if s22 and s32:
            desired["LF2"] = True

        if s12 and not desired["LS2"]:
            desired["HS12"] = True
        if s22 and not (desired["LS2"] or desired["LF2"]):
            desired["HS22"] = True
        if s32 and not desired["LF2"]:
            desired["HF2"] = True

        # 5) Persist hysteresis memory
        self._last_dark = dark

        if self.debug:
            print("[CoveragePlanner] sector_darkness:", [round(x, 1) for x in sector_darkness])
            print("[CoveragePlanner] dark flags     :", dark)
            print("[CoveragePlanner] desired_on     :", desired)

        return desired


# -------------------------- Convenience function API --------------------------

# A module-level singleton to keep hysteresis state if callers use the function.
_PLANNER_SINGLETON: Optional[CoveragePlanner] = None

def plan_desired_on(
    sector_darkness: List[float],
    t_on: float = 12.0,
    t_off: float = 8.0,
    debug: bool = False
) -> Dict[str, bool]:
    """
    Convenience functional-style API that keeps per-sector hysteresis internally.

    Example:
        desired = plan_desired_on(cell_darkness, t_on=12.0, t_off=8.0)
    """
    global _PLANNER_SINGLETON
    if _PLANNER_SINGLETON is None:
        _PLANNER_SINGLETON = CoveragePlanner(T_on=t_on, T_off=t_off, debug=debug)
    else:
        # If thresholds change at runtime, update them
        _PLANNER_SINGLETON.T_on = t_on
        _PLANNER_SINGLETON.T_off = t_off
        _PLANNER_SINGLETON.debug = debug

    return _PLANNER_SINGLETON.update(sector_darkness)
