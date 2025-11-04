# modules/EmergencyController.py
# Handles UPS edge detection and EMERGENCY mode transitions.
# Behavior:
# - If you switch back to AUTO while UPS is already On Battery => ENTER EMERGENCY.
# - In AUTO: when UPS transitions OL -> OB => ENTER EMERGENCY.
# - In AUTO: when UPS returns to OL => EXIT EMERGENCY.
# - NEW: If the system boots in AUTO while UPS is already OB => ENTER EMERGENCY.

from typing import Optional, Tuple


def _has_token(status_str: str, token: str) -> bool:
    """Return True if 'token' appears as a space/comma-separated token in status_str."""
    if not status_str:
        return False
    toks = status_str.replace(",", " ").lower().split()
    return token.lower() in toks


def _is_online(status_str: Optional[str]) -> bool:
    """UPS Online (OL, possibly with suffix like 'CHRG')."""
    if not status_str:
        return False
    return _has_token(status_str, "ol")


def _is_on_battery(status_str: Optional[str]) -> bool:
    """UPS On Battery (OB, possibly with suffix like 'DISCHRG')."""
    if not status_str:
        return False
    return _has_token(status_str, "ob")


class EmergencyController:
    """
    Tracks previous UPS status and current emergency flag.

    Call eval_transition(...) once per loop with the current UPS status and mode flags.

    Returns (emergency_mode, event), where event is one of:
      - "enter_boot_on_battery"  (booted in AUTO while UPS already OB)
      - "enter_returned_to_auto" (switched to AUTO while UPS already OB)
      - "enter_online_to_onbatt" (OL -> OB while AUTO)
      - "exit_online"            (OB -> OL while AUTO and emergency True)
      - None                     (no state change)
    """

    def __init__(self) -> None:
        self.prev_ups_status: Optional[str] = None
        self.emergency: bool = False

    def eval_transition(
        self,
        ups_status_str: str,
        auto_mode: bool,
        just_switched_to_auto: bool,
    ) -> Tuple[bool, Optional[str]]:
        status = (ups_status_str or "").upper()

        now_online = _is_online(status)
        now_on_batt = _is_on_battery(status)
        was_online = _is_online(self.prev_ups_status) if self.prev_ups_status is not None else False
        booting = self.prev_ups_status is None

        event: Optional[str] = None

        # Case 0 (NEW): Boot in AUTO while already on battery -> enter emergency
        if auto_mode and now_on_batt and not self.emergency and booting:
            self.emergency = True
            event = "enter_boot_on_battery"

        # Case 1: Returned to AUTO while UPS already on battery -> enter emergency
        elif just_switched_to_auto and now_on_batt and not self.emergency:
            self.emergency = True
            event = "enter_returned_to_auto"

        # Case 2: In AUTO, detect OL -> OB edge -> enter emergency
        elif auto_mode and (not self.emergency) and was_online and now_on_batt:
            self.emergency = True
            event = "enter_online_to_onbatt"

        # Case 3: In AUTO, if currently emergency and UPS back Online -> exit emergency
        elif auto_mode and self.emergency and now_online:
            self.emergency = False
            event = "exit_online"

        # Update prev status AFTER decisions
        self.prev_ups_status = status
        return self.emergency, event
