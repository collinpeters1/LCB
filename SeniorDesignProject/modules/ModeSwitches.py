# modules/ModeSwitches.py
import time
import RPi.GPIO as GPIO

AUTO_PIN_DEFAULT = 22  # HIGH => AUTO
DN_PIN_DEFAULT   = 23  # external pull network, leave internal pull OFF

class ModeSwitches:
    def __init__(self, auto_pin=AUTO_PIN_DEFAULT, dn_pin=DN_PIN_DEFAULT, dn_debounce_ms=75):
        self.auto_pin = auto_pin
        self.dn_pin = dn_pin
        self.dn_debounce_s = dn_debounce_ms / 1000.0

        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.auto_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.dn_pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)

        self._dn_last_stable = self._read_dn_raw()
        self._dn_candidate = self._dn_last_stable
        self._dn_last_change_time = time.monotonic()

    def _read_auto_raw(self) -> bool:
        return GPIO.input(self.auto_pin) == GPIO.HIGH  # HIGH => AUTO

    def _read_dn_raw(self) -> bool:
        return GPIO.input(self.dn_pin) == GPIO.HIGH

    def read_auto(self) -> bool:
        return self._read_auto_raw()

    def read_dn_debounced(self):
        """
        Returns (dn_state, changed)
        """
        raw = self._read_dn_raw()
        now = time.monotonic()

        if raw != self._dn_candidate:
            self._dn_candidate = raw
            self._dn_last_change_time = now
            changed = False
        else:
            if (now - self._dn_last_change_time) >= self.dn_debounce_s and self._dn_candidate != self._dn_last_stable:
                self._dn_last_stable = self._dn_candidate
                changed = True
            else:
                changed = False

        return self._dn_last_stable, changed

    def read_states(self):
        auto_mode = self.read_auto()
        dn_state, dn_changed = self.read_dn_debounced()
        return {"auto_mode": auto_mode, "dn_state": dn_state, "dn_changed": dn_changed}

    def cleanup(self):
        GPIO.cleanup()
