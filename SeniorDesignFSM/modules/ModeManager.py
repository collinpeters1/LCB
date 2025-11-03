#!/usr/bin/env python3
"""
ModeManager.py — Read panel Auto/Manual and Day/Night switches with debounce.

Responsibilities
---------------
- Configure GPIO for:
    * AUTO/MANUAL input: returns True when MANUAL is selected (active-low by default).
    * DAY/NIGHT input  : debounced level (True=Day, False=Night by default).
- Expose simple methods used by main.py:
    * is_manual() -> bool
    * read_dn_debounced() -> (bool level_is_day, bool changed_since_last_call)
- Provide cleanup() to release GPIO safely.

Conventions (can be overridden via Config.py)
--------------------------------------------
- Pull resistors: BOARD wiring often sets external resistors; internally we default to PUD_UP.
- Logic levels (defaults, override in Config if your board differs):
    MANUAL_ACTIVE_LOW = True   # manual when pin is LOW
    DAY_HIGH_IS_DAY   = True   # day when pin is HIGH

Debounce
--------
- Day/Night is debounced in software with a simple "stable for X ms" filter.
- Manual is read as level (no debounce by default), but you can enable one if needed.

Requires
--------
- RPi.GPIO (already used in your project)
"""

from dataclasses import dataclass
from typing import Tuple, Optional
import time
import RPi.GPIO as GPIO

try:
    from modules import Config as C
except Exception:
    C = None


@dataclass
class _DNState:
    last_raw: Optional[int] = None
    stable_level: Optional[int] = None
    last_change_t: float = 0.0
    stable_since_t: float = 0.0


class ModeManager:
    def __init__(
        self,
        *,
        auto_pin: int,
        dn_pin: int,
        dn_debounce_ms: int = 50,
        manual_active_low: Optional[bool] = None,
        day_high_is_day: Optional[bool] = None,
        use_internal_pullups: bool = True,
    ):
        """
        :param auto_pin: BCM pin wired to the Auto/Manual selector (center-off safe).
        :param dn_pin:   BCM pin wired to Day/Night selector (center-off safe).
        :param dn_debounce_ms: Debounce window for Day/Night (milliseconds).
        :param manual_active_low: If True, LOW=Manual; if False, HIGH=Manual.
        :param day_high_is_day:   If True, HIGH=Day; else LOW=Day.
        :param use_internal_pullups: If True, enable PUD_UP on inputs (safe default).
        """
        self.auto_pin = auto_pin
        self.dn_pin = dn_pin
        self.dn_debounce_s = max(0.0, dn_debounce_ms / 1000.0)

        # Defaults from Config.py or sane fallbacks
        self.manual_active_low = (
            (getattr(C, "MANUAL_ACTIVE_LOW", True) if manual_active_low is None else manual_active_low)
        )
        self.day_high_is_day = (
            (getattr(C, "DAY_HIGH_IS_DAY", True) if day_high_is_day is None else day_high_is_day)
        )

        # --- GPIO setup
        GPIO.setmode(GPIO.BCM)
        pud = GPIO.PUD_UP if use_internal_pullups else GPIO.PUD_OFF

        GPIO.setup(self.auto_pin, GPIO.IN, pull_up_down=pud)
        GPIO.setup(self.dn_pin, GPIO.IN, pull_up_down=pud)

        # Day/Night debounce state
        self._dn = _DNState()
        # Initialize DN stable state immediately from current level
        init_level = GPIO.input(self.dn_pin)
        self._dn.last_raw = init_level
        self._dn.stable_level = init_level
        t = time.monotonic()
        self._dn.last_change_t = t
        self._dn.stable_since_t = t

    # ------------------ Public API ------------------

    def is_manual(self) -> bool:
        """
        Return True when the panel is in MANUAL mode according to wiring logic.
        Default: pin LOW means Manual (active-low).
        """
        lvl = GPIO.input(self.auto_pin)
        if self.manual_active_low:
            return (lvl == GPIO.LOW)
        else:
            return (lvl == GPIO.HIGH)

    def read_dn_debounced(self) -> Tuple[bool, bool]:
        """
        Debounced read of Day/Night selector.

        :return: (is_day, changed)
                 is_day  = logical day/night after mapping (True=Day).
                 changed = True when the debounced stable state changed since last call.
        """
        raw = GPIO.input(self.dn_pin)
        now = time.monotonic()

        if self._dn.last_raw is None:
            self._dn.last_raw = raw
            self._dn.last_change_t = now

        changed = False

        # Edge on raw input → reset debounce timer
        if raw != self._dn.last_raw:
            self._dn.last_raw = raw
            self._dn.last_change_t = now

        # If raw input has been stable for >= debounce window, commit it as stable
        if (now - self._dn.last_change_t) >= self.dn_debounce_s:
            if raw != self._dn.stable_level:
                self._dn.stable_level = raw
                self._dn.stable_since_t = now
                changed = True

        # Map to logical "is_day"
        is_day = (self._dn.stable_level == GPIO.HIGH) if self.day_high_is_day else (self._dn.stable_level == GPIO.LOW)
        return bool(is_day), bool(changed)

    def cleanup(self) -> None:
        """
        Release GPIO (safe to call multiple times).
        """
        try:
            GPIO.cleanup(self.auto_pin)
        except Exception:
            pass
        try:
            GPIO.cleanup(self.dn_pin)
        except Exception:
            pass
