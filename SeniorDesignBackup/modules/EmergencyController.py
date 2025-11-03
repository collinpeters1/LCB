# modules/EmergencyController.py
# Handles UPS edge detection and EMERGENCY mode transitions.
# Mirrors your existing behavior:
# - If you switch back to AUTO while UPS is already On Battery => ENTER EMERGENCY.
# - In AUTO: when UPS transitions OL -> OB => ENTER EMERGENCY.
# - In AUTO: when UPS returns to OL => EXIT EMERGENCY.

from typing import Optional, Tuple

def _has_token(status_str: str, token: str) -> bool:
    if not status_str:
        return False
    toks = status_str.replace(",", " ").lower().split()
    return token.lower() in toks

def _is_online(status_str: str) -> bool:
    return _has_token(status_str, "ol")   # matches "OL", "OL CHRG"

def _is_on_battery(status_str: str) -> bool:
    return _has_token(status_str, "ob")   # matches "OB", "OB DISCHRG"

class EmergencyController:
    """
    Tracks previous UPS status and current emergency flag.
    Call eval_transition(...) once per loop with the current UPS status and mode flags.
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
        """
        Evaluate and update emergency state.
        Returns (emergency_mode, event), where event is one of:
          - "enter_returned_to_auto" (AUTO entered while UPS already OB)
          - "enter_online_to_onbatt" (OL -> OB while AUTO)
          - "exit_online"            (OB -> OL while AUTO and emergency True)
          - None                     (no state change)

        Your main.py should:
          - if event startswith("enter"): call reset_all(dac)
          - print the same messages you used before based on event
        """
        status = (ups_status_str or "").upper()
        now_online  = _is_online(status)
        now_on_batt = _is_on_battery(status)
        was_online  = _is_online(self.prev_ups_status) if self.prev_ups_status is not None else False

        event: Optional[str] = None

        # Case 1: Returned to AUTO while UPS already on battery -> enter emergency
        if just_switched_to_auto and now_on_batt and not self.emergency:
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
