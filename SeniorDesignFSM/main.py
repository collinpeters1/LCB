#!/usr/bin/env python3
"""
main.py — ICLS orchestrator (FSM version, CSI-only) with dual views & hotkeys

Views:
  - "ICLS HDR"      : HDR frame with right-side info (UPS/mode/DAC/params)
  - "ICLS Analysis" : Analyzer-annotated grid (S11..S32 with %dark)

Hotkeys:
  [  ]  : analyzer threshold  -/+   (0..100)
   -  = : T_on (allowable %dark to turn ON)  -/+  (0..100)
   \    : toggle global Otsu for analyzer threshold (on/off)
   p    : print current params to console
   h    : toggle on-screen help overlay
   q    : quit
"""

# --- Disable OpenCV's GStreamer backend BEFORE importing cv2 (safety) ---
import os as _os
_os.environ["OPENCV_VIDEOIO_PRIORITY_GSTREAMER"] = "0"

# ---------- stdlib ----------
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import time
import cv2

# ---------- project config & modules ----------
from modules import Config as C
from modules.LightingAnalysis import LightingAnalyzer
from modules.UPS import get_ups_data
from modules.OverlayRenderer import draw_analysis

# FSM & IO modules
from modules.ModeManager import ModeManager
from modules.CameraRouter import CameraRouter
from modules.CoveragePlanner import plan_desired_on
from modules.LightingFSM import LightFSM, FSMParams
from modules.DACBus import DACBus


# =========================== Data containers ===========================

@dataclass
class UPSStatus:
    raw: str = "OL CHRG"              # e.g. "OL CHRG" or "OB DISCHRG"
    is_battery: bool = False
    charge_pct: Optional[float] = None
    runtime_s: Optional[int] = None


@dataclass
class SystemMode:
    # Precedence: Emergency (UPS OB) > Manual (panel) > Auto
    emergency: bool
    manual: bool
    auto: bool
    day_mode: bool  # True = Day camera, False = Night camera

    @property
    def label(self) -> str:
        return "EMERGENCY" if self.emergency else ("MANUAL" if self.manual else "AUTO")


@dataclass
class LightingState:
    dac: Dict[str, int] = field(default_factory=dict)      # per-light 0..255
    desired_on: Dict[str, bool] = field(default_factory=dict)
    fsm: Dict[str, "LightFSM"] = field(default_factory=dict)


# =========================== Helpers ===========================

def _parse_ups(raw: Dict[str, str]) -> UPSStatus:
    status = raw.get("ups.status", "OL CHRG")
    is_batt = status.startswith("OB")
    try:
        charge = float(raw.get("battery.charge", "0"))
    except Exception:
        charge = None
    try:
        runtime = int(float(raw.get("battery.runtime", "0")))
    except Exception:
        runtime = None
    return UPSStatus(raw=status, is_battery=is_batt, charge_pct=charge, runtime_s=runtime)


def _sector_signal_for_light(light: str, sector_dark: List[float]) -> float:
    """
    Darkness signal each light should react to. Order: [S11,S12,S21,S22,S31,S32]
    """
    S11, S12, S21, S22, S31, S32 = 0, 1, 2, 3, 4, 5

    # Column 1
    if light == "HS11": return sector_dark[S11]
    if light == "HS21": return sector_dark[S21]
    if light == "LS1":  return max(sector_dark[S11], sector_dark[S21])
    if light == "LF1":  return max(sector_dark[S21], sector_dark[S31])
    if light == "HF1":  return sector_dark[S31]

    # Column 2
    if light == "HS12": return sector_dark[S12]
    if light == "HS22": return sector_dark[S22]
    if light == "LS2":  return max(sector_dark[S12], sector_dark[S22])
    if light == "LF2":  return max(sector_dark[S22], sector_dark[S32])
    if light == "HF2":  return sector_dark[S32]

    return 0.0


def _init_fsms(light_names: List[str]) -> Dict[str, "LightFSM"]:
    p = FSMParams(
        T_on=getattr(C, "DARK_T_ON", 12.0),
        T_off=getattr(C, "DARK_T_OFF", 8.0),
        hold_margin=getattr(C, "DARK_T_HOLD_MARGIN", 1.0),
        ramp_up=getattr(C, "RAMP_UP_CPS", 240),
        ramp_down=getattr(C, "RAMP_DOWN_CPS", 120),
        min_on_s=getattr(C, "MIN_ON_S", 0.6),
        min_off_s=getattr(C, "MIN_OFF_S", 0.8),
        off_guard_s=getattr(C, "OFF_GUARD_S", 0.15),
        rearm_s=getattr(C, "REARM_S", 0.25),
        cap_on=getattr(C, "DAC_MAX", 255),
        floor=0,
        enable_trim=getattr(C, "FSM_TRIM_ENABLE", True),
        trim_deadband=getattr(C, "FSM_TRIM_DB", 1.5),
        trim_kp=getattr(C, "FSM_TRIM_KP", 4.0),
        trim_rate_limit=getattr(C, "FSM_TRIM_RATE", 40),
        debug=getattr(C, "FSM_DEBUG", False),
    )
    return {name: LightFSM(params=p, name=name) for name in light_names}


def _help_lines(thresh: float, t_on: float, t_off: float, otsu: bool) -> List[str]:
    return [
        "Hotkeys:",
        "[ ]  analyzer threshold -/+",
        "- =  T_on (allowable %dark) -/+",
        "\\    toggle Otsu for analyzer",
        "p    print params",
        "h    toggle this help",
        "q    quit",
        "",
        f"Analyzer threshold: {thresh:.1f}%  (Otsu={'ON' if otsu else 'OFF'})",
        f"T_on: {t_on:.1f}%   T_off: {t_off:.1f}%",
    ]


# =========================== Main ===========================

def main():
    # ---- timing & constants
    LOOP_PERIOD_S = getattr(C, "LOOP_PERIOD_S", 0.05)  # ~20 Hz default
    DAC_MAX = getattr(C, "DAC_MAX", 255)

    # ---- lights (all 10, canonical order)
    LIGHTS = getattr(
        C, "LIGHT_NAMES",
        ["HS11","HS21","LS1","LF1","HF1","HS12","HS22","LS2","LF2","HF2"]
    )

    # ---- live-tunable params (start from Config)
    analyzer_threshold = float(getattr(C, "THRESHOLD_DARK", 10.0))  # % (0..100)
    t_on = float(getattr(C, "DARK_T_ON", 12.0))
    t_off = float(getattr(C, "DARK_T_OFF", 8.0))
    hold_margin = float(getattr(C, "DARK_T_HOLD_MARGIN", 1.0))
    use_otsu = False
    show_help = True

    # ---- managers
    mode_mgr = ModeManager(
        auto_pin=getattr(C, "AUTO_PIN", 22),
        dn_pin=getattr(C, "DN_PIN", 23),
        dn_debounce_ms=getattr(C, "DN_DEBOUNCE_MS", 50),
    )
    cam = CameraRouter(
        day_name=getattr(C, "HDR_DAY_NAME", "IMX708"),
        night_name=getattr(C, "HDR_NIGHT_NAME", getattr(C, "HDR_NIGHT_NAME", "IMX290")),
        settle_s=getattr(C, "SWITCH_SETTLE_S", 0.20),
        verbose=getattr(C, "CAM_ROUTER_VERBOSE", False),
    )
    analyzer = LightingAnalyzer(
        threshold_value=analyzer_threshold,
        rows=3, cols=2,
        ema_alpha=getattr(C, "EMA_ALPHA", 0.7)
    )
    dac = DACBus.from_config(C)

    # ---- state
    state = LightingState(
        dac={name: 0 for name in LIGHTS},
        desired_on={name: False for name in LIGHTS},
        fsm=_init_fsms(LIGHTS),
    )

    print("ICLS (FSM) orchestrator running — press 'q' to quit.")
    print("Hotkeys: [ ] threshold  |  - = T_on  |  \\ Otsu  |  h help  |  p print  |  q quit")

    last_t = time.monotonic()

    try:
        while True:
            loop_t0 = time.monotonic()

            # 1) UPS + panel switches
            ups = _parse_ups(get_ups_data() or {})
            is_day, _changed = mode_mgr.read_dn_debounced()
            is_manual = mode_mgr.is_manual()
            mode = SystemMode(
                emergency=ups.is_battery,
                manual=(not ups.is_battery) and is_manual,
                auto=(not ups.is_battery) and (not is_manual),
                day_mode=is_day
            )

            # 2) Cameras (route HDR by Day/Night)
            cam.ensure_active("Day" if mode.day_mode else "Night")
            hdr_frame = cam.get_hdr_frame()

            # 3) Analysis (sector %dark) on HDR frame
            analysis_frame = hdr_frame

            # Optional global Otsu for analyzer threshold
            if use_otsu and analysis_frame is not None:
                hsv = cv2.cvtColor(analysis_frame, cv2.COLOR_BGR2HSV)
                v = hsv[:, :, 2]
                _thr, _ = cv2.threshold(v, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                # Map 0..255 to 0..100%
                analyzer.threshold_value = float(_thr) * 100.0 / 255.0
            else:
                analyzer.threshold_value = analyzer_threshold

            # Compute darkness and get annotated analysis view
            sector_dark, analysis_annot = analyzer.compute_darkness_compact(analysis_frame)

            # 4/5/6) Control paths
            if mode.emergency:
                # UPS on battery → Emergency rule: all off except LF2 = max
                for name, fsm in state.fsm.items():
                    if name == "LF2":
                        fsm.force(DAC_MAX, new_state="HOLD")
                        state.dac[name] = DAC_MAX
                    else:
                        fsm.force(0, new_state="OFF")
                        state.dac[name] = 0
                dac.write_all(state.dac)

            elif mode.manual:
                # MANUAL: hold last DAC values (panel adjustments external, if any)
                pass

            else:
                # AUTO: planner + per-light FSMs
                state.desired_on = plan_desired_on(
                    sector_dark,
                    t_on=t_on,
                    t_off=t_off,
                    debug=getattr(C, "PLANNER_DEBUG", False),
                )

                # FSM step for each light → new DAC
                dt = max(0.0, loop_t0 - last_t)
                for name, fsm in state.fsm.items():
                    # keep FSM params (hold_margin) in sync if changed
                    fsm.params.hold_margin = hold_margin
                    want = state.desired_on.get(name, False)
                    signal = _sector_signal_for_light(name, sector_dark)
                    state.dac[name] = fsm.step(want=want, darkness=signal, dt=dt)

                # Batch push to both DACs
                dac.write_all(state.dac)

            # 7) Overlays & windows
            # Right-side info for HDR view
            if hdr_frame is not None:
                ups_text = ups.raw
                mode_text = mode.label
                right_info = [
                    f"Mode: {mode_text}",
                    f"UPS: {ups_text}",
                    f"Analyzer thr: {analyzer.threshold_value:.1f}%  (Otsu={'ON' if use_otsu else 'OFF'})",
                    f"T_on: {t_on:.1f}%   T_off: {t_off:.1f}%",
                ]
                # Show DACs in canonical order
                right_info += [f"{k}:{state.dac.get(k,0):3d}" for k in LIGHTS]
                if show_help:
                    right_info += ["", *(_help_lines(analyzer.threshold_value, t_on, t_off, use_otsu))]

                out_hdr = draw_analysis(
                    hdr_frame,
                    cell_darkness=sector_dark,
                    info_right=right_info,
                    grid_rows=3, grid_cols=2
                )
                cv2.imshow("ICLS HDR", out_hdr)

            # Separate analysis view (annotated grid)
            if analysis_annot is not None:
                # Add a small text header with the key params
                annot = analysis_annot.copy()
                hdr = f"thr={analyzer.threshold_value:.1f}%  (Otsu={'ON' if use_otsu else 'OFF'})   T_on={t_on:.1f}%  T_off={t_off:.1f}%"
                cv2.putText(annot, hdr, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (240, 240, 240), 2, cv2.LINE_AA)
                cv2.imshow("ICLS Analysis", annot)

            # 8) Keyboard + timing
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('h'):
                show_help = not show_help
            elif key == ord('p'):
                print(f"[Params] analyzer_threshold={analyzer.threshold_value:.1f}% "
                      f"(Otsu={'ON' if use_otsu else 'OFF'}), T_on={t_on:.1f}%, T_off={t_off:.1f}%")
            elif key == ord('['):
                analyzer_threshold = max(0.0, analyzer_threshold - 1.0)
            elif key == ord(']'):
                analyzer_threshold = min(100.0, analyzer_threshold + 1.0)
            elif key == ord('-'):
                t_on = max(0.0, t_on - 0.5)
                # keep hysteresis sensible: ensure T_off <= T_on - hold_margin (clamp)
                t_off = min(t_off, max(0.0, t_on - hold_margin))
            elif key == ord('='):
                t_on = min(100.0, t_on + 0.5)
                # re-allow room for hysteresis; don't auto-raise t_off unless needed
                if t_off > t_on - hold_margin:
                    t_off = max(0.0, t_on - hold_margin)
            elif key == ord('\\'):
                use_otsu = not use_otsu

            last_t = loop_t0
            elapsed = time.monotonic() - loop_t0
            time.sleep(max(0.0, LOOP_PERIOD_S - elapsed))

    except KeyboardInterrupt:
        pass

    finally:
        # Graceful shutdown
        try:
            dac.write_all({name: 0 for name in LIGHTS})
        except Exception:
            pass
        try:
            dac.close()
        except Exception:
            pass
        try:
            cam.release()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            mode_mgr.cleanup()
        except Exception:
            pass


if __name__ == "__main__":
    main()
