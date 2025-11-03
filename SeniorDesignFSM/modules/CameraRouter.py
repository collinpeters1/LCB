#!/usr/bin/env python3
"""
CameraRouter.py — Minimal day/night HDR camera router (Pi 5, CSI).

Responsibilities
----------------
- Own two HDR-capable CSI cameras (e.g., IMX708 for Day, IMX327 for Night).
- Lazy-open each camera the first time it becomes active.
- Provide a clean API used by main.py:
    * ensure_active("Day" | "Night")
    * get_hdr_frame() -> np.ndarray | None
    * get_active_name() -> str
    * release()

Compatibility
-------------
- Works whether DualPicam.PicamFeed exposes .get_frame() or .capture_frame().
  (We check with hasattr and call whichever exists.)

Optional Config hooks (modules/Config.py)
-----------------------------------------
HDR_DAY_NAME   = "IMX708"
HDR_NIGHT_NAME = "IMX327"
CAM_NAME_TO_NUM = {"IMX708": 0, "IMX327": 1}
CAM_PARAMS      = {"IMX708": {"af": True}, "IMX327": {"af": False}}
SWITCH_SETTLE_S = 0.20
"""

from typing import Optional, Dict, Any
import time

try:
    from modules.DualPicam import PicamFeed
except Exception as e:
    raise ImportError("CameraRouter requires modules.DualPicam.PicamFeed") from e

try:
    from modules import Config as C
except Exception:
    C = None


class CameraRouter:
    """
    Router for two HDR CSI cameras (Day/Night).
    """

    def __init__(
        self,
        *,
        day_name: Optional[str] = None,
        night_name: Optional[str] = None,
        settle_s: Optional[float] = None,
        close_inactive: bool = False,
        verbose: bool = False,
    ):
        self.verbose = bool(verbose)

        # Labels used in overlays/logging
        self.day_name = day_name or (getattr(C, "HDR_DAY_NAME", "IMX708") if C else "IMX708")
        self.night_name = night_name or (getattr(C, "HDR_NIGHT_NAME", "IMX327") if C else "IMX327")

        # Mapping of label -> camera_num
        self.name_to_num: Dict[str, int] = (
            getattr(C, "CAM_NAME_TO_NUM", {"IMX708": 0, "IMX327": 1})
            if C else {"IMX708": 0, "IMX327": 1}
        )

        # Optional per-camera kwargs for PicamFeed (e.g., autofocus)
        self.cam_params: Dict[str, Dict[str, Any]] = (
            getattr(C, "CAM_PARAMS", {"IMX708": {"af": True}, "IMX327": {"af": False}})
            if C else {"IMX708": {"af": True}, "IMX327": {"af": False}}
        )

        # Settle delay after switching
        self.settle_s = (
            settle_s
            if settle_s is not None
            else (getattr(C, "SWITCH_SETTLE_S", 0.20) if C else 0.20)
        )

        self.CLOSE_INACTIVE = bool(close_inactive)

        # Internal handles (lazy-opened)
        self._day_cam: Optional[PicamFeed] = None
        self._night_cam: Optional[PicamFeed] = None

        # Start with Day active by default (caller will call ensure_active)
        self._active_label: str = "Day"
        self._last_switch_t: float = 0.0

    # ---------------- Public API ----------------

    def ensure_active(self, label: str) -> None:
        """
        Ensure the requested label ("Day" or "Night") is currently active.
        If this is a change, lazily open the needed camera and optionally
        close the inactive one. A short settle delay is enforced after switching.
        """
        if label not in ("Day", "Night"):
            raise ValueError("ensure_active expects 'Day' or 'Night'")

        if label == self._active_label:
            return

        if label == "Day":
            self._ensure_open(self.day_name)
            if self.CLOSE_INACTIVE and self._night_cam is not None:
                self._night_cam.release()
                self._night_cam = None
        else:
            self._ensure_open(self.night_name)
            if self.CLOSE_INACTIVE and self._day_cam is not None:
                self._day_cam.release()
                self._day_cam = None

        self._active_label = label
        self._last_switch_t = time.monotonic()
        if self.verbose:
            print(f"[CameraRouter] Switched active to {self.get_active_name()}")
        if self.settle_s > 0.0:
            time.sleep(self.settle_s)

    def get_hdr_frame(self):
        """
        Capture and return a frame from the active HDR camera (numpy array).
        Returns None if the active camera isn’t ready yet.
        """
        cam = self._active_cam()
        if cam is None:
            self._ensure_open(self.day_name if self._active_label == "Day" else self.night_name)
            cam = self._active_cam()
            if cam is None:
                return None

        # Compatibility: prefer get_frame(); if absent, use capture_frame()
        if hasattr(cam, "get_frame"):
            return cam.get_frame()
        elif hasattr(cam, "capture_frame"):
            return cam.capture_frame()
        else:
            raise AttributeError("PicamFeed lacks both get_frame() and capture_frame()")

    def get_active_name(self) -> str:
        """Human-readable camera name: e.g., 'IMX708 (Day)' or 'IMX327 (Night)'."""
        return f"{self.day_name} (Day)" if (self._active_label == "Day") else f"{self.night_name} (Night)"

    def release(self) -> None:
        """Close any opened camera handles."""
        try:
            if self._day_cam is not None:
                self._day_cam.release()
        finally:
            self._day_cam = None
        try:
            if self._night_cam is not None:
                self._night_cam.release()
        finally:
            self._night_cam = None

    # --------------- Internals ---------------

    def _active_cam(self) -> Optional[PicamFeed]:
        return self._day_cam if (self._active_label == "Day") else self._night_cam

    def _ensure_open(self, name: str) -> None:
        """Open (or no-op if already open) the PicamFeed for the given camera name."""
        cam_num = self.name_to_num.get(name, 0)
        params = dict(self.cam_params.get(name, {}))  # shallow copy
        params.setdefault("af", True)

        if name == self.day_name:
            if self._day_cam is None:
                self._day_cam = PicamFeed(camera_num=cam_num, **params)
        elif name == self.night_name:
            if self._night_cam is None:
                self._night_cam = PicamFeed(camera_num=cam_num, **params)
        else:
            # If a third label appears in Config, put it in the Night slot by convention
            if self._night_cam is None:
                self._night_cam = PicamFeed(camera_num=cam_num, **params)
