# modules/OverlayRenderer.py
# Draws the analysis-window overlay exactly like your current main.py:
# - Centered MODE banner (AUTO/MANUAL)
# - Left-side Column 1 DACs
# - Right-side Column 2 DACs

import cv2

def draw_analysis(analysis_img, dac: dict, auto_mode: bool):
    """
    Mutates and returns analysis_img with overlay drawn.

    Params:
      analysis_img : np.ndarray (BGR)
      dac          : dict with keys:
                     HS11, HS21, LS1, HF1, LF1, HS12, HS22, LS2, HF2, LF2
      auto_mode    : bool
    """
    if analysis_img is None:
        return analysis_img

    H, W = analysis_img.shape[:2]
    font  = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.7
    thick = 2
    left_x = 10
    top_y  = 30
    line_h = 30
    color  = (0, 255, 0)

    def put_right(text, y):
        (tw, th), _ = cv2.getTextSize(text, font, scale, thick)
        x = W - 10 - tw
        cv2.putText(analysis_img, text, (x, y), font, scale, color, thick)

    # Mode banner centered at top
    mode_text = "MODE: AUTO" if auto_mode else "MODE: MANUAL"
    (tw, th), _ = cv2.getTextSize(mode_text, font, 0.9, 2)
    cv2.putText(analysis_img, mode_text, ((W - tw)//2, 25), font, 0.9, (0, 255, 0), 2)

    # Column 1 (left)
    cv2.putText(analysis_img, f"HS11: {dac['HS11']}", (left_x, top_y + 0*line_h), font, scale, color, thick)
    cv2.putText(analysis_img, f"HS21: {dac['HS21']}", (left_x, top_y + 1*line_h), font, scale, color, thick)
    cv2.putText(analysis_img, f"HF1:  {dac['HF1']}",  (left_x, top_y + 2*line_h), font, scale, color, thick)
    cv2.putText(analysis_img, f"LS1:  {dac['LS1']}",  (left_x, top_y + 3*line_h), font, scale, color, thick)
    cv2.putText(analysis_img, f"LF1:  {dac['LF1']}",  (left_x, top_y + 4*line_h), font, scale, color, thick)

    # Column 2 (right)
    put_right(f"HS12: {dac['HS12']}", top_y + 0*line_h)
    put_right(f"HS22: {dac['HS22']}", top_y + 1*line_h)
    put_right(f"HF2:  {dac['HF2']}",  top_y + 2*line_h)
    put_right(f"LS2:  {dac['LS2']}",  top_y + 3*line_h)
    put_right(f"LF2:  {dac['LF2']}",  top_y + 4*line_h)

    return analysis_img
