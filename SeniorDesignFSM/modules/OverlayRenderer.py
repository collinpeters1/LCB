#!/usr/bin/env python3
"""
OverlayRenderer.py — Draws the grid, per-cell %darkness, and a right-side info panel.

New API (used by the new main.py)
---------------------------------
    render_overlay(
        frame,                        # np.ndarray BGR image (HDR frame)
        cell_darkness,                # list[6] of %dark in order [S11,S12,S21,S22,S31,S32]
        info_right=None,              # list[str] lines to print on the right side
        grid_rows=3, grid_cols=2,     # grid layout (default 3x2)
        title="ICLS"                  # optional window title/text banner
    ) -> np.ndarray

Back-compat shim (used by older scripts)
----------------------------------------
    draw_analysis(frame, arg2, arg3)
    - If arg2 is a dict (DAC map), assumes legacy usage and shows DACs + AUTO/MANUAL.
    - If arg2 looks like a 6-element list (cell darkness), it forwards to the new API.

Notes
-----
- This module has NO hardware I/O — purely image annotation.
- Coordinates and fonts are conservative to be readable on 1080p and Pi displays.
"""

from typing import List, Optional, Sequence, Dict
import cv2
import numpy as np

# -------- Helpers: text & box drawing --------

def _put_text(img, text, org, font_scale=0.6, thickness=1, color=(255,255,255)):
    cv2.putText(img, str(text), org, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness, cv2.LINE_AA)

def _draw_panel(img, lines: Sequence[str], x_right: int, top: int, line_h: int = 20):
    """Draw right-side text panel with a subtle background."""
    if not lines:
        return
    # Compute panel width from longest line (approximate character width)
    max_chars = max(len(s) for s in lines)
    panel_w = int(max(180, min(380, max_chars * 8)))
    left = max(0, x_right - panel_w - 10)
    right = x_right - 10
    bottom = top + 10 + line_h * (len(lines) + 1)
    overlay = img.copy()
    cv2.rectangle(overlay, (left, top), (right, bottom), (0, 0, 0), thickness=-1)
    # semi-transparent
    cv2.addWeighted(overlay, 0.35, img, 0.65, 0.0, dst=img)
    # Text
    y = top + 24
    for s in lines:
        _put_text(img, s, (left + 12, y), font_scale=0.6, thickness=1, color=(255,255,255))
        y += line_h

def _grid_labels(rows: int, cols: int) -> List[str]:
    # S11, S12, S21, S22, S31, S32 (row-major)
    labels = []
    for r in range(1, rows+1):
        for c in range(1, cols+1):
            labels.append(f"S{r}{c}")
    return labels

def _format_pct(x: float) -> str:
    try:
        return f"{float(x):.1f}%"
    except Exception:
        return "—"


# -------- New API --------

def render_overlay(
    frame: "np.ndarray",
    cell_darkness: Optional[Sequence[float]] = None,
    info_right: Optional[Sequence[str]] = None,
    grid_rows: int = 3,
    grid_cols: int = 2,
    title: str = "ICLS",
) -> "np.ndarray":
    """
    Draw a 3x2 grid, annotate each cell with its %darkness, and show a right-side info panel.
    Returns a new annotated frame (does not modify input in-place).
    """
    if frame is None:
        return frame
    img = frame.copy()
    h, w = img.shape[:2]

    # Title banner (top-left)
    _put_text(img, title, (12, 28), font_scale=0.8, thickness=2, color=(240, 240, 240))

    # Draw grid
    cell_w = w // grid_cols
    cell_h = h // grid_rows
    for c in range(1, grid_cols):
        x = c * cell_w
        cv2.line(img, (x, 0), (x, h), (128, 128, 128), 1, cv2.LINE_AA)
    for r in range(1, grid_rows):
        y = r * cell_h
        cv2.line(img, (0, y), (w, y), (128, 128, 128), 1, cv2.LINE_AA)

    # Per-cell labels and %darkness
    if cell_darkness is not None and len(cell_darkness) == grid_rows * grid_cols:
        labels = _grid_labels(grid_rows, grid_cols)  # ["S11","S12",...]
        idx = 0
        for r in range(grid_rows):
            for c in range(grid_cols):
                x0, y0 = c * cell_w, r * cell_h
                # cell label (top-left of the cell)
                _put_text(img, labels[idx], (x0 + 8, y0 + 22), font_scale=0.7, thickness=2, color=(255, 255, 0))
                # %dark value (bottom-left of the cell)
                _put_text(img, _format_pct(cell_darkness[idx]), (x0 + 8, y0 + cell_h - 12), font_scale=0.7, thickness=2, color=(255, 255, 255))
                idx += 1

    # Right-side info panel
    if info_right:
        _draw_panel(img, info_right, x_right=w - 8, top=10)

    return img


# -------- Legacy-compatible wrapper --------

def draw_analysis(frame, arg2=None, arg3=None, *args, **kwargs):
    """
    Backward-compatible entry point.

    Legacy usage (old code):
        draw_analysis(frame, dac_dict, auto_mode_bool)
            -> shows a right panel with Mode and DACs

    New usage (new main.py):
        draw_analysis(frame, cell_darkness=[...], info_right=[...], grid_rows=3, grid_cols=2)
            -> forwarded to render_overlay(...)
    """
    # Heuristics to detect legacy signature:
    # - arg2 is a dict (DAC mapping), arg3 is bool (auto/man)
    if isinstance(arg2, dict):
        dac: Dict[str, int] = arg2
        auto_mode = bool(arg3) if arg3 is not None else True
        lines = [f"Mode: {'AUTO' if auto_mode else 'MANUAL'}"]
        # Sorted DACs for stable order
        for k in sorted(dac.keys()):
            v = dac[k]
            lines.append(f"{k}:{int(v):3d}")
        return render_overlay(frame, cell_darkness=None, info_right=lines, grid_rows=3, grid_cols=2, title="ICLS")

    # Otherwise, assume new signature (cell_darkness & info_right in kwargs or positionals)
    cell_darkness = arg2 if (arg2 is not None) else kwargs.get("cell_darkness")
    info_right = arg3 if (arg3 is not None) else kwargs.get("info_right")
    grid_rows = kwargs.get("grid_rows", 3)
    grid_cols = kwargs.get("grid_cols", 2)
    title = kwargs.get("title", "ICLS")
    return render_overlay(frame, cell_darkness=cell_darkness, info_right=info_right, grid_rows=grid_rows, grid_cols=grid_cols, title=title)
