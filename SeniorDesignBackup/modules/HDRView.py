# modules/HDRView.py
import time
import cv2
import numpy as np
from modules.DualPicam import PicamFeed

def _safe_stop(cam):
    try:
        if cam:
            cam.stop()
    except Exception:
        pass

def ensure_active(
    desired_name: str,
    active_name: str,
    hdr_cam_primary,
    hdr_cam_alt,
    settle_s: float = 0.08,
):
    """
    Robust, reuse-first switching:
      - If already on desired_name -> ensure that feed exists and is started; stop the other one.
      - Else: stop the current feed, small settle, then start (or create+start) the target.
      - Keep the non-target feed allocated for re-use.
      - Retry once with a longer settle if the target start throws 'busy'.
    Returns (new_active_name, hdr_cam_primary, hdr_cam_alt).
    """

    def _start_target():
        nonlocal hdr_cam_primary, hdr_cam_alt
        if desired_name == "IMX708":
            if hdr_cam_primary is None:
                hdr_cam_primary = PicamFeed(camera_num=0, af=True)
            hdr_cam_primary.start()
            _safe_stop(hdr_cam_alt)
        else:  # "IMX327"
            if hdr_cam_alt is None:
                hdr_cam_alt = PicamFeed(camera_num=1, af=False)
            hdr_cam_alt.start()
            _safe_stop(hdr_cam_primary)

    # Case 1: Names already match — make sure feed actually exists and is running.
    if desired_name == active_name:
        try:
            _start_target()
        except Exception as e:
            # Retry once after a slightly longer settle (in case startup races)
            time.sleep(max(settle_s, 0.2))
            try:
                _start_target()
            except Exception:
                print(f"[Cam] Ensure '{desired_name}' failed to start: {e}")
        return active_name, hdr_cam_primary, hdr_cam_alt

    # Case 2: Real switch — stop current, settle, then start target.
    if active_name == "IMX708":
        _safe_stop(hdr_cam_primary)
    elif active_name == "IMX327":
        _safe_stop(hdr_cam_alt)

    if settle_s and settle_s > 0:
        time.sleep(settle_s)

    try:
        _start_target()
    except Exception as e:
        time.sleep(max(settle_s, 0.2))
        try:
            _start_target()
        except Exception:
            print(f"[Cam] Switch to {desired_name} failed: {e}")
            return active_name, hdr_cam_primary, hdr_cam_alt

    print(f"[Cam] Switched to {desired_name} ({'Day' if desired_name=='IMX708' else 'Night'}).")
    return desired_name, hdr_cam_primary, hdr_cam_alt


def capture_frame(hdr_cam):
    if hdr_cam is None:
        return None
    try:
        return hdr_cam.capture_frame()
    except Exception:
        return None

def _has_token(status_str: str, token: str) -> bool:
    if not status_str:
        return False
    toks = status_str.replace(",", " ").lower().split()
    return token.lower() in toks

def _is_on_battery(status_str: str) -> bool:
    return _has_token(status_str, "ob")

def _put_hdr_overlay(frame, *, active_name: str, auto_mode: bool, ups_data: dict, emergency_mode: bool):
    if frame is None:
        return frame
    if len(frame.shape) == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    current_status  = ups_data.get("ups.status", "Unknown")
    current_charge  = ups_data.get("battery.charge", "Unknown")
    current_runtime = ups_data.get("battery.runtime", "0")
    try:
        runtime_sec = int(current_runtime)
    except Exception:
        runtime_sec = 0
    h, m, s = runtime_sec // 3600, (runtime_sec % 3600) // 60, runtime_sec % 60
    formatted_runtime = f"{h:02}:{m:02}:{s:02}"

    cv2.putText(frame, f"Active: {active_name}",         (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
    cv2.putText(frame, f"UPS Status: {current_status}",  (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    cv2.putText(frame, f"Charge: {current_charge}%",     (10, 90),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    cv2.putText(frame, f"Runtime: {formatted_runtime}",  (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    mode_text_hdr = "MODE: AUTO" if auto_mode else "MODE: MANUAL"
    cv2.putText(frame, mode_text_hdr, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    if auto_mode and emergency_mode:
        cv2.putText(frame, "EMERGENCY MODE", (10, 185),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 3)
    elif not auto_mode and _is_on_battery(current_status):
        cv2.putText(frame, "EMERGENCY OVERRIDDEN BY MANUAL", (10, 185),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0,165,255), 3)
    return frame

def show(hdr_frame, *, active_name: str, auto_mode: bool, ups_data: dict, emergency_mode: bool, window_name: str = "HDR Feed (CSI)"):
    if hdr_frame is None:
        return
    frame = _put_hdr_overlay(hdr_frame, active_name=active_name, auto_mode=auto_mode,
                             ups_data=ups_data, emergency_mode=emergency_mode)
    try:
        cv2.imshow(window_name, frame)
    except Exception:
        pass
