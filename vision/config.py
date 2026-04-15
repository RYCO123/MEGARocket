from .models import CameraModel, TelemetrySample

BENCH_CAMERA_HEIGHT_IN = 31.5
BENCH_CAMERA_HEIGHT_M = BENCH_CAMERA_HEIGHT_IN * 0.0254
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

# Bench-calibrated effective FOV for the current Walksnail capture path.
# This is narrower than the published lens spec and better matches the
# recorded image geometry in the current setup.
DEFAULT_CAMERA_MODEL_NAME = "CADDXFPV Walksnail Avatar Pro Kit"
WALKSNAIL_AVATAR_PRO_CAMERA_MODEL = CameraModel(
    horizontal_fov_deg=118.3,
    vertical_fov_deg=105.7,
)

DEFAULT_CAMERA_MODEL = WALKSNAIL_AVATAR_PRO_CAMERA_MODEL


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
