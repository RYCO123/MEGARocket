import csv
import math
from pathlib import Path

from .models import CameraRelativeCoordinate, Detection, TelemetrySample

# Default path for the calibration file produced by calibrate_lens.py
CALIBRATION_CSV = Path(__file__).resolve().parent / "results" / "calibration.csv"


class CalibratedProjector:
    """
    Projects a pixel detection to a camera-relative ground coordinate using a
    pre-collected lens calibration table.

    Camera mounting: the camera is rotated 90° CCW (viewed from above), so:
        image +U (right)  →  craft forward (nose)
        image +V (down)   →  craft right (starboard)

    Output coordinate system (camera frame convention):
        +x  →  right (starboard)
        +y  →  forward (nose)

    Because image U = craft forward and image V = craft right:
        x_m = altitude * tan(theta_V(|dy|) * sign(dy) + roll_rad)
        y_m = altitude * tan(theta_U(|dx|) * sign(dx) + pitch_rad)
    """

    def __init__(self, calibration_csv: Path = CALIBRATION_CSV):
        self._x_pixels: list = []
        self._x_thetas: list = []
        self._y_pixels: list = []
        self._y_thetas: list = []
        self._load(Path(calibration_csv))

    def _load(self, path: Path):
        if not path.exists():
            raise FileNotFoundError(
                f"Calibration file not found: {path}\n"
                f"Run:  python3 vision/calibrate_lens.py --height-cm <HEIGHT>"
            )
        with open(path, newline='', encoding='utf-8') as f:
            for row in csv.DictReader(f):
                axis = row['axis']
                px = float(row['pixel_offset_px'])
                theta = float(row['theta_rad'])
                if axis == 'x':
                    self._x_pixels.append(px)
                    self._x_thetas.append(theta)
                elif axis == 'y':
                    self._y_pixels.append(px)
                    self._y_thetas.append(theta)
        if not self._x_pixels or not self._y_pixels:
            raise ValueError(f"Calibration file missing x or y data: {path}")

    @staticmethod
    def _interp(offsets: list, thetas: list, abs_px: float) -> float:
        """Linear interpolation (and extrapolation) of theta for a pixel offset."""
        if abs_px <= 0.0:
            return 0.0
        # Below the first calibrated point: interpolate linearly from origin
        if abs_px <= offsets[0]:
            return thetas[0] * (abs_px / offsets[0])
        # Between two known points
        for i in range(len(offsets) - 1):
            if offsets[i] <= abs_px <= offsets[i + 1]:
                t = (abs_px - offsets[i]) / (offsets[i + 1] - offsets[i])
                return thetas[i] + t * (thetas[i + 1] - thetas[i])
        # Beyond the last calibrated point: linear extrapolation
        slope = (thetas[-1] - thetas[-2]) / (offsets[-1] - offsets[-2])
        return thetas[-1] + slope * (abs_px - offsets[-1])

    def project(self, detection: Detection, frame_shape, telemetry: TelemetrySample):
        if telemetry.altitude_m <= 0:
            return None

        h, w = frame_shape[:2]
        dx = detection.center_x_px - w / 2.0
        dy = detection.center_y_px - h / 2.0

        theta_x = self._interp(self._x_pixels, self._x_thetas, abs(dx))
        theta_y = self._interp(self._y_pixels, self._y_thetas, abs(dy))

        if dx < 0:
            theta_x = -theta_x
        if dy < 0:
            theta_y = -theta_y

        # image U (dx) = craft forward → pitch tilts it → drives y_m
        # image V (dy) = craft right  → roll tilts it  → drives x_m
        theta_x += math.radians(telemetry.pitch_deg)
        theta_y += math.radians(telemetry.roll_deg)

        return CameraRelativeCoordinate(
            x_m=telemetry.altitude_m * math.tan(theta_y),  # V offset → craft right
            y_m=telemetry.altitude_m * math.tan(theta_x),  # U offset → craft forward
            z_m=telemetry.altitude_m,
        )


class NullProjector:
    """Stand-in projector used when no calibration file is available."""

    def project(self, detection, frame_shape, telemetry):
        return None
