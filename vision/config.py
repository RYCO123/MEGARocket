from .models import TelemetrySample

BENCH_CAMERA_HEIGHT_IN = 31.5
BENCH_CAMERA_HEIGHT_M = BENCH_CAMERA_HEIGHT_IN * 0.0254
BENCH_CAMERA_HEIGHT_CM = BENCH_CAMERA_HEIGHT_IN * 2.54
BENCH_TARGET_OFFSET_IN = 21.0
BENCH_TARGET_OFFSET_M = BENCH_TARGET_OFFSET_IN * 0.0254


TARGET_COLOR_RANGES = {
    "blue": [
        ((100, 85, 120), (112, 255, 255)),
    ],
    "yellow": [
        ((18, 100, 100), (38, 255, 255)),
    ],
    "green": [
        ((35, 80, 50), (90, 255, 255)),
    ],
    "orange": [
        ((5, 120, 80), (25, 255, 255)),
    ],
    "red": [
        ((0, 145, 130), (18, 255, 255)),
        ((176, 190, 180), (180, 255, 255)),
    ],
}

DEFAULT_TARGET_COLOR = "red"
DEFAULT_TARGET_MIN_AREA_PX = 800

TELEMETRY_SCENARIOS = {
    "bench_nadir": TelemetrySample(
        altitude_m=BENCH_CAMERA_HEIGHT_M,
        pitch_deg=0.0,
        roll_deg=0.0,
        yaw_deg=0.0,
    ),
    "bench_pitch_5": TelemetrySample(
        altitude_m=BENCH_CAMERA_HEIGHT_M,
        pitch_deg=5.0,
        roll_deg=0.0,
        yaw_deg=0.0,
    ),
    "bench_roll_5": TelemetrySample(
        altitude_m=BENCH_CAMERA_HEIGHT_M,
        pitch_deg=0.0,
        roll_deg=5.0,
        yaw_deg=0.0,
    ),
}
