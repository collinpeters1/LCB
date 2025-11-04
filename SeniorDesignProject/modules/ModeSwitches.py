# SeniorDesignProject/modules/ModeSwitches.py
# Raspberry Pi 5 ONLY — uses rpi-lgpio directly (no RPi.GPIO).
# Exposes the same ModeSwitches API your code already calls.

import time
import lgpio  # pip install rpi-lgpio

class ModeSwitches:
    """
    Reads front-panel switches using BCM pin numbers on Raspberry Pi 5.
      - auto_pin: HIGH = AUTO, LOW = MANUAL (assumes pull-up)
      - dn_pin:   momentary/toggle, debounced in software
    """
    def __init__(self, auto_pin=22, dn_pin=23, dn_debounce_ms=50, chip_index=0):
        self.auto_pin = int(auto_pin)
        self.dn_pin = int(dn_pin)
        self.dn_debounce_ms = int(dn_debounce_ms)

        # Open gpiochip (usually 0 on Pi 5)
        self._h = lgpio.gpiochip_open(chip_index)
        self._claimed = set()

        # Configure as inputs with internal pull-ups
        self._setup_input(self.auto_pin, pull_up=True)
        self._setup_input(self.dn_pin,   pull_up=True)

        # Debounce state
        self._last_dn = self.read_dn()
        self._last_t = time.time()

    # ---------- low-level helpers ----------
    def _setup_input(self, pin, pull_up=True):
        flags = lgpio.SET_PULL_UP if pull_up else 0
        # Claim the line as input with optional pull-up
        lgpio.gpio_claim_input(self._h, pin, flags)
        self._claimed.add(pin)

    def _read(self, pin) -> int:
        return 1 if lgpio.gpio_read(self._h, pin) else 0

    # ---------- public API (kept same) ----------
    def read_dn(self) -> int:
        return self._read(self.dn_pin)

    def read_states(self):
        auto = self._read(self.auto_pin)  # 1 = HIGH (AUTO), 0 = LOW (MANUAL)
        dn_now = self.read_dn()
        t = time.time()
        changed = False
        # simple time-based debounce
        if dn_now != self._last_dn and (t - self._last_t) * 1000.0 >= self.dn_debounce_ms:
            changed = True
            self._last_dn = dn_now
            self._last_t = t
        return {"auto_mode": auto, "dn_state": dn_now, "dn_changed": changed}

    def read_dn_debounced(self):
        s = self.read_states()
        return s["dn_state"], s["dn_changed"]

    def cleanup(self):
        try:
            lgpio.gpiochip_close(self._h)
        except Exception:
            pass
