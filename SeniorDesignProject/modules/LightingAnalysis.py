# LightingAnalysis.py
# Supports OV9281 brightness analysis with toggleable thresholding:
#   - "global": fixed threshold (manual / adjustable)
#   - "local_otsu": per-cell Otsu threshold (rows × cols)

import cv2
import numpy as np


class CameraManager:
    """Handles capture from a USB/UVC camera (e.g., OV9281)."""
    def __init__(self, camera_index=0, width=1280, height=720, fps=60):
        self.camera_index = camera_index
        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise Exception(f"Cannot open camera at index {self.camera_index}")

        # Request MJPG stream
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        got_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        got_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        got_fps = self.cap.get(cv2.CAP_PROP_FPS)
        print(f"Camera initialized on index {self.camera_index} at {got_w}x{got_h} @ {got_fps:.1f} fps")

    def capture_frame(self):
        ok, frame = self.cap.read()
        return frame if ok else None

    def release(self):
        self.cap.release()
        cv2.destroyAllWindows()
        print("Camera released.")


class LightingAnalyzer:
    """
    Computes per-sector darkness on a grid and draws an overlay.

    Modes:
      • 'global'     – fixed threshold_value
      • 'local_otsu' – Otsu computed separately in each grid cell
    """
    def __init__(self, threshold_value=30, rows=3, cols=2):
        self.threshold_value = int(threshold_value)
        self.rows = rows
        self.cols = cols
        self.threshold_mode = "global"  # default mode

    # ---------------- public controls ----------------
    def adjust_threshold(self, delta: int):
        self.threshold_value = int(np.clip(self.threshold_value + int(delta), 0, 255))
        print(f"[LightingAnalyzer] threshold → {self.threshold_value}")

    def get_threshold(self) -> int:
        return int(self.threshold_value)

    def set_threshold_mode(self, mode: str):
        mode = str(mode).lower()
        if mode not in ("global", "local_otsu"):
            print(f"[LightingAnalyzer] Unknown mode '{mode}'")
            return
        self.threshold_mode = mode
        print(f"[LightingAnalyzer] mode → {self.threshold_mode}")
    # -------------------------------------------------

    def _brightness_map(self, frame):
        """Return a single-channel brightness map (uint8)."""
        if len(frame.shape) == 2:
            v = frame
        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            v = hsv[:, :, 2]
        return v.astype(np.uint8)

    def _threshold_global(self, v):
        t = int(np.clip(self.threshold_value, 0, 255))
        _, mask = cv2.threshold(v, t, 255, cv2.THRESH_BINARY)
        return mask

    def _threshold_local_otsu(self, v):
        """Apply Otsu independently within each grid cell."""
        h, w = v.shape
        cell_h = h // self.rows
        cell_w = w // self.cols
        mask = np.zeros_like(v, dtype=np.uint8)

        for r in range(self.rows):
            for c in range(self.cols):
                y0, y1 = r * cell_h, (r + 1) * cell_h if r < self.rows - 1 else h
                x0, x1 = c * cell_w, (c + 1) * cell_w if c < self.cols - 1 else w
                tile = v[y0:y1, x0:x1]

                # Use Otsu unless tile nearly uniform
                if int(tile.max()) - int(tile.min()) < 5:
                    _, tile_mask = cv2.threshold(tile, self.threshold_value, 255, cv2.THRESH_BINARY)
                else:
                    _, tile_mask = cv2.threshold(tile, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                mask[y0:y1, x0:x1] = tile_mask
        return mask

    def analyze(self, frame):
        v = self._brightness_map(frame)

        # Select thresholding method
        if self.threshold_mode == "local_otsu":
            mask = self._threshold_local_otsu(v)
        else:
            mask = self._threshold_global(v)

        # Prepare overlay background
        annotated = cv2.cvtColor(v, cv2.COLOR_GRAY2BGR)

        total = v.size
        bright = int(np.count_nonzero(mask))
        overall_dark = 100.0 - (bright / total * 100.0) if total else 0.0

        # Grid split
        h, w = v.shape
        ch, cw = h // self.rows, w // self.cols
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (0, 155, 255)
        cell_dark = []

        for r in range(self.rows):
            for c in range(self.cols):
                y0, y1 = r * ch, (r + 1) * ch if r < self.rows - 1 else h
                x0, x1 = c * cw, (c + 1) * cw if c < self.cols - 1 else w
                cell = mask[y0:y1, x0:x1]
                total_pix = cell.size
                bright_pix = int(np.count_nonzero(cell))
                darkness = 100.0 - (bright_pix / total_pix * 100.0) if total_pix else 0.0
                cell_dark.append(darkness)

                text = f"{darkness:.1f}%"
                (tw, th), _ = cv2.getTextSize(text, font, 0.9, 3)
                cx = x0 + (cw - tw) // 2
                cy = y0 + (ch + th) // 2
                cv2.putText(annotated, text, (cx, cy), font, 0.9, color, 3)

        # Draw grid lines
        for r in range(1, self.rows):
            cv2.line(annotated, (0, r * ch), (w, r * ch), color, 3)
        for c in range(1, self.cols):
            cv2.line(annotated, (c * cw, 0), (c * cw, h), color, 3)

        # HUD
        #hud = f"Mode={self.threshold_mode}  T={self.threshold_value}"
        #cv2.putText(annotated, hud, (10, 25), font, 0.8, (255, 255, 255), 2)

        return overall_dark, cell_dark, annotated
