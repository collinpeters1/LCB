#!/usr/bin/env python3
"""
DualPicam.py — Picamera2-only camera feed wrapper for CSI sensors (no OpenCV fallback)

Provides:
    class PicamFeed:
        PicamFeed(camera_num: int = 0, af: bool = True, width=None, height=None, fps=None)
        get_frame() -> np.ndarray | None
        capture_frame() -> np.ndarray | None   # alias of get_frame()
        release() -> None

Behavior:
- Use Picamera2 exclusively when available. No OpenCV fallback on your Pi.
- If Picamera2 import fails (unexpected here), minimal OpenCV fallback with a warning.

Notes:
- Frames are returned in BGR (OpenCV convention).
- Autofocus flag (af) is applied when supported.
"""

from typing import Optional
import time
import numpy as np

# Prefer Picamera2 on Pi
try:
    from picamera2 import Picamera2
    HAVE_PICAM2 = True
except Exception:
    HAVE_PICAM2 = False

# OpenCV is only used for color conversion and (rare) fallback
import cv2


class PicamFeed:
    def __init__(
        self,
        camera_num: int = 0,
        *,
        af: bool = True,
        width: Optional[int] = None,
        height: Optional[int] = None,
        fps: Optional[int] = None,
    ):
        """
        :param camera_num: index of the CSI sensor (as seen by Picamera2)
        :param af: enable autofocus if supported
        :param width, height, fps: desired stream properties (best-effort)
        """
        self._use_picam2 = False
        self._pc2: Optional[Picamera2] = None
        self._cv2cap: Optional[cv2.VideoCapture] = None

        self._af = bool(af)
        self._width = width
        self._height = height
        self._fps = fps

        if HAVE_PICAM2:
            # Picamera2-only path — no OpenCV fallback when Picamera2 is present
            self._init_picam2_strict(camera_num)
            self._use_picam2 = True
        else:
            # Absolute last resort — you shouldn't hit this on your Pi 5 setup
            print("[DualPicam] WARNING: Picamera2 not available; falling back to OpenCV VideoCapture.")
            self._init_cv2(camera_num)

    # -------------------- Public API --------------------

    def get_frame(self) -> Optional["np.ndarray"]:
        """Return a single BGR frame, or None on failure."""
        if self._use_picam2 and self._pc2 is not None:
            try:
                # Picamera2 returns RGB; convert to BGR
                arr = self._pc2.capture_array("main")
                if arr is None or arr.size == 0:
                    return None
                return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            except Exception:
                return None
        else:
            if self._cv2cap is None:
                return None
            ok, frame = self._cv2cap.read()
            return frame if ok else None

    # Back-compat alias
    def capture_frame(self) -> Optional["np.ndarray"]:
        return self.get_frame()

    def release(self) -> None:
        """Release underlying resources; safe to call multiple times."""
        if self._pc2 is not None:
            try:
                self._pc2.stop()
            except Exception:
                pass
            try:
                self._pc2.close()
            except Exception:
                pass
            self._pc2 = None

        if self._cv2cap is not None:
            try:
                self._cv2cap.release()
            except Exception:
                pass
            self._cv2cap = None

    # -------------------- Internals --------------------

    def _init_picam2_strict(self, camera_num: int) -> None:
        """
        Initialize Picamera2 and *never* fall back to OpenCV when Picamera2 is present.
        Raises a descriptive exception if camera_num is invalid or configuration fails.
        """
        # Validate camera index against available sensors
        try:
            infos = Picamera2.global_camera_info()
        except Exception as e:
            raise RuntimeError(f"[DualPicam] Picamera2 not ready: {e}")

        if not infos:
            raise RuntimeError("[DualPicam] No CSI cameras detected by Picamera2.")

        if camera_num < 0 or camera_num >= len(infos):
            models = ", ".join(i.get("Model", "?") for i in infos)
            raise ValueError(
                f"[DualPicam] Invalid camera_num {camera_num}. "
                f"Available indices: 0..{len(infos)-1} ({models})"
            )

        # Create and configure preview stream
        pc2 = Picamera2(camera_num)

        # IMPORTANT: Only pass 'main' when we have a dict; otherwise call without it.
        if self._width and self._height:
            main_cfg = {"size": (int(self._width), int(self._height))}
            config = pc2.create_preview_configuration(main=main_cfg)
        else:
            config = pc2.create_preview_configuration()

        pc2.configure(config)

        # Attempt autofocus if supported/desired
        try:
            if self._af:
                cam_ctrl = pc2.camera_controls
                if "AfMode" in cam_ctrl:
                    pc2.set_controls({"AfMode": 2})  # 2 = Continuous
                elif "AfTrigger" in cam_ctrl:
                    pc2.set_controls({"AfTrigger": 0})
        except Exception:
            pass

        # Optional FPS tuning (best-effort via controls)
        if self._fps:
            try:
                pc2.set_controls({"FrameRate": float(self._fps)})
            except Exception:
                pass

        # Start stream
        pc2.start()
        time.sleep(0.03)  # tiny warm-up

        self._pc2 = pc2

    def _init_cv2(self, camera_num: int) -> None:
        """
        Minimal OpenCV fallback only used when Picamera2 import failed.
        """
        cap = cv2.VideoCapture(int(camera_num), cv2.CAP_V4L2)
        if self._width:  cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self._width))
        if self._height: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self._height))
        if self._fps:    cap.set(cv2.CAP_PROP_FPS, int(self._fps))
        self._cv2cap = cap
