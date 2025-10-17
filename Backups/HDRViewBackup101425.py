# modules/HDRView.py
import cv2
import numpy as np
from modules.DualPicam import PicamFeed

def ensure_active(desired_name: str, active_name: str, hdr_cam_primary, hdr_cam_alt):
    if desired_name == active_name:
        try:
            if desired_name == "IMX708":
                if hdr_cam_primary: hdr_cam_primary.start()
                if hdr_cam_alt:     hdr_cam_alt.stop()
            else:
                if hdr_cam_alt:     hdr_cam_alt.start()
                if hdr_cam_primary: hdr_cam_primary.stop()
        except Exception:
            pass
        return active_name, hdr_cam_primary, hdr_cam_alt

    if desired_name == "IMX708":
        try:
            if hdr_cam_alt: hdr_cam_alt.stop()
        except Exception:
            pass
        if hdr_cam_primary is None:
            hdr_cam_primary = PicamFeed(camera_num=0, af=True)
        else:
            try:
                hdr_cam_primary.start()
            except Exception:
                pass
        print("[Cam] Switched to IMX708 (Day).")
        return "IMX708", hdr_cam_primary, hdr_cam_alt

    try:
        if hdr_cam_primary: hdr_cam_primary.stop()
    except Exception:
        pass
    if hdr_cam_alt is None:
        hdr_cam_alt = PicamFeed(camera_num=1, af=False)
    else:
        try:
            hdr_cam_alt.start()
        except Exception:
            pass
    print("[Cam] Switched to IMX327 (Night).")
    return "IMX327", hdr_cam_primary, hdr_cam_alt


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
    return _has_token(status_str, "ob")   # matches "OB", "OB DISCHRG"


def _put_hdr_overlay(frame, *, active_name: str, auto_mode: bool, ups_data: dict, emergency_mode: bool):
    if frame is None:
        return frame

    if len(frame.shape) == 2:
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    current_status  = ups_data.get("ups.status", "Unknown")
    current_charge  = ups_data.get("battery.charge", "Unknown")
    current_runtime = ups_data.get("battery.runtime", "0")

    # Format runtime (sec->HH:MM:SS)
    try:
        runtime_sec = int(current_runtime)
    except Exception:
        runtime_sec = 0
    h, m, s = runtime_sec // 3600, (runtime_sec % 3600) // 60, runtime_sec % 60
    formatted_runtime = f"{h:02}:{m:02}:{s:02}"

    # Static overlays
    cv2.putText(frame, f"Active: {active_name}",         (10, 30),  cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
    cv2.putText(frame, f"UPS Status: {current_status}",  (10, 60),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    cv2.putText(frame, f"Charge: {current_charge}%",     (10, 90),  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    cv2.putText(frame, f"Runtime: {formatted_runtime}",  (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    mode_text_hdr = "MODE: AUTO" if auto_mode else "MODE: MANUAL"
    cv2.putText(frame, mode_text_hdr, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

    # Corrected logic:
    # - In AUTO: show "EMERGENCY MODE" when emergency flag is true.
    # - In MANUAL: show "EMERGENCY OVERRIDDEN BY MANUAL" whenever UPS is on battery (OB),
    #   even if the internal emergency flag wasn't set while in manual.
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
