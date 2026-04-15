from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class CameraModel:
    horizontal_fov_deg: float
    vertical_fov_deg: float


@dataclass(frozen=True)
class TelemetrySample:
    altitude_m: float
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    center_x_px: float
    center_y_px: float
    area_px: float
    x_px: int
    y_px: int
    width_px: int
    height_px: int


@dataclass(frozen=True)
class CameraRelativeCoordinate:
    x_m: float
    y_m: float
    z_m: float


@dataclass(frozen=True)
class TrackingResult:
    detection: Optional[Detection]
    coordinate: Optional[CameraRelativeCoordinate]
    telemetry: TelemetrySample
    camera_model: CameraModel
    suggested_camera_model: Optional[CameraModel] = None
