# modules/FloodMap.py
#
# Per-light output shaping for DAC values.
#
# Problem:
#   Flood lights don't respond at all below ~63 DAC,
#   then slam near-max between ~63-80, and above ~80 they
#   basically look identical (saturated).
#
# Goal:
#   Stretch that tiny "useful" window [63..80] across our whole
#   logical 0..255 range so the controller (PID etc.) still thinks
#   in 0..255, but the hardware only ever sees 63..80.
#
# Spots/others should stay 1:1 (no shaping).
#
# Tuning knobs:
FLOOD_MIN_DAC = 60   # first code where flood is visibly on
FLOOD_MAX_DAC = 100   # code where flood already looks "full"
# You can tweak those two numbers on the bench without touching anything else.

# Which lights are "flood behavior" and need mapping?
FLOOD_LIGHTS = {"HF1", "HF2", "LF1", "LF2"}

def _scale_linear(val, in_lo, in_hi, out_lo, out_hi):
    """
    Map val in [in_lo..in_hi] to [out_lo..out_hi], clamp both ends.
    All ints in/out.
    """
    if val < in_lo:
        val = in_lo
    if val > in_hi:
        val = in_hi
    in_span = in_hi - in_lo
    out_span = out_hi - out_lo
    if in_span <= 0:
        return out_lo
    scaled = out_lo + (val - in_lo) * out_span / in_span
    return int(round(scaled))

def map_light_output(light_name: str, logical_val: int) -> int:
    """
    Given a light name (e.g. 'HF1') and the controller's desired 0..255 value,
    return the DAC code we should actually send.

    Floods: stretch 0..255 -> FLOOD_MIN_DAC..FLOOD_MAX_DAC.
    Others: passthrough.
    """
    # clamp logical to [0..255]
    if logical_val < 0:
        logical_val = 0
    if logical_val > 255:
        logical_val = 255

    if light_name in FLOOD_LIGHTS:
        return _scale_linear(
            logical_val,
            0,   255,
            FLOOD_MIN_DAC,
            FLOOD_MAX_DAC
        )
    else:
        # spots / other lights: unchanged
        return logical_val

def map_state(state: dict[str, int]) -> dict[str, int]:
    """
    Take the whole light state dict like {"HS11": 12, "HF1": 200, ...}
    and return a NEW dict with per-light shaping applied.
    """
    out = {}
    for name, v in state.items():
        out[name] = map_light_output(name, int(v))
    return out
