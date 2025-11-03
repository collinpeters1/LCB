#!/usr/bin/env python3
"""
LightingAnalysis.py — Capture an analysis frame and compute %dark per 3×2 grid cell.
(Updated: force V4L2 backend, robust VideoCapture init with index auto-scan and clear logs)
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import cv2
import numpy as np

try:
    from modules import Config as C
except Exception:
    C = None


# ----------------------------- Camera wrapper -----------------------------

class CameraManager:
    """
    Thin wrapper for an analysis camera using OpenCV VideoCapture, with robust init.
    - Forces the V4L2 backend (avoids GStreamer pipeline issues).
    - Tries the configured ANALYSIS_CAMERA_INDEX first (default 0).
    - If that fails, auto-scans indices 0..5 and picks the first that opens.
    """

    def __init__(self, camera_index: int = 0, width: Optional[int] = None, height: Optional[int] = None):
        self.cap: Optional[cv2.VideoCapture] = None
        self.index_tried: List[int] = []

        # Preferred index from Config or arg
        pref = camera_index
        if C is not None:
            pref = getattr(C, "ANALYSIS_CAMERA_INDEX", pref)

        # Try preferred
        self._try_open(pref, width, height)
        # If not opened, scan 0..5 (skip the one we already tried)
        if not self._is_open():
            for idx in range(0, 6):
                if idx == pref:
                    continue
                self._try_open(idx, width, height)
                if self._is_open():
                    print(f"[LightingAnalysis] Opened analysis camera at index {idx} (V4L2)")
                    break

        if not self._is_open():
            tried = ", ".join(map(str, self.index_tried))
            print(f"[LightingAnalysis] WARNING: could not open any analysis camera (tried: {tried}). "
                  f"Will fall back to HDR frame for analysis when available.")

    def _try_open(self, idx: int, width: Optional[int], height: Optional[int]) -> None:
        self.index_tried.append(idx)
        # Force V4L2 backend explicitly
        cap = cv2.VideoCapture(int(idx), cv2.CAP_V4L2)
        if width:  cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        if height: cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        ok = cap.isOpened()
        if ok:
            self.cap = cap
        else:
            cap.release()

    def _is_open(self) -> bool:
        return self.cap is not None and self.cap.isOpened()

    def read(self) -> Optional["np.ndarray"]:
        if not self._is_open():
            return None
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self) -> None:
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        self.cap = None


# ----------------------------- Analyzer core -----------------------------

@dataclass
class LightingAnalyzer:
    """
    Compute per-cell darkness over a 3×2 grid on the HSV V-channel.
    """
    threshold_value: float = 10.0  # % dark threshold (0..100)
    rows: int = 3
    cols: int = 2
    ema_alpha: Optional[float] = None
    _ema_cells: List[float] = field(default_factory=list)

    # Legacy API
    def compute_darkness(self, frame: Optional["np.ndarray"]) -> Tuple[float, List[float], "np.ndarray"]:
        overall, cells, annotated = self._analyze(frame)
        return overall, cells, annotated

    # Compact API used by main.py
    def compute_darkness_compact(self, frame: Optional["np.ndarray"]) -> Tuple[List[float], "np.ndarray"]:
        _, cells, annotated = self._analyze(frame)
        return cells, annotated

    # Internals
    def _analyze(self, frame: Optional["np.ndarray"]) -> Tuple[float, List[float], "np.ndarray"]:
        if frame is None or frame.size == 0:
            # Blank frame fallback
            cells = [0.0] * (self.rows * self.cols)
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            return 0.0, cells, blank

        h, w = frame.shape[:2]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        v = hsv[:, :, 2]  # 0..255

        v_thresh = int(np.clip(self.threshold_value, 0.0, 100.0) * 255.0 / 100.0)

        cell_h = h // self.rows
        cell_w = w // self.cols

        per_cell: List[float] = []
        for r in range(self.rows):
            for c in range(self.cols):
                y0, y1 = r * cell_h, (r + 1) * cell_h if r < self.rows - 1 else h
                x0, x1 = c * cell_w, (c + 1) * cell_w if c < self.cols - 1 else w
                cell_v = v[y0:y1, x0:x1]
                if cell_v.size == 0:
                    per_cell.append(0.0)
                    continue
                dark_mask = (cell_v <= v_thresh)
                dark_pct = float(dark_mask.sum()) * 100.0 / float(cell_v.size)
                per_cell.append(dark_pct)

        # EMA smoothing
        if self.ema_alpha is not None and 0.0 < self.ema_alpha <= 1.0:
            if not self._ema_cells or len(self._ema_cells) != len(per_cell):
                self._ema_cells = per_cell[:]
            else:
                a = self.ema_alpha
                self._ema_cells = [a * x + (1.0 - a) * y for x, y in zip(per_cell, self._ema_cells)]
            per_cell_smoothed = self._ema_cells[:]
        else:
            per_cell_smoothed = per_cell

        overall = float(np.mean(per_cell_smoothed)) if per_cell_smoothed else 0.0

        annotated = frame.copy()
        self._draw_grid_and_labels(annotated, per_cell_smoothed)

        return overall, per_cell_smoothed, annotated

    def _draw_grid_and_labels(self, img: "np.ndarray", per_cell: List[float]) -> None:
        h, w = img.shape[:2]
        cell_h = h // self.rows
        cell_w = w // self.cols

        for c in range(1, self.cols):
            x = c * cell_w
            cv2.line(img, (x, 0), (x, h), (128, 128, 128), 1, cv2.LINE_AA)
        for r in range(1, self.rows):
            y = r * cell_h
            cv2.line(img, (0, y), (w, y), (128, 128, 128), 1, cv2.LINE_AA)

        idx = 0
        for r in range(self.rows):
            for c in range(self.cols):
                x0, y0 = c * cell_w, r * cell_h
                s_label = f"S{r+1}{c+1}"
                cv2.putText(img, s_label, (x0 + 8, y0 + 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2, cv2.LINE_AA)
                try:
                    pct_text = f"{per_cell[idx]:.1f}%"
                except Exception:
                    pct_text = "—"
                cv2.putText(img, pct_text, (x0 + 8, y0 + cell_h - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                idx += 1
