import math

from .models import CameraModel, CameraRelativeCoordinate, Detection, TelemetrySample


class GroundProjector:
    def __init__(self, camera_model: CameraModel):
        self.camera_model = camera_model

    def project(self, detection: Detection, frame_shape, telemetry: TelemetrySample):
        return self.project_with_camera_model(
            detection=detection,
            frame_shape=frame_shape,
            telemetry=telemetry,
            camera_model=self.camera_model,
        )

    def project_with_camera_model(self, detection: Detection, frame_shape, telemetry: TelemetrySample, camera_model: CameraModel):
        frame_height, frame_width = frame_shape[:2]
        if telemetry.altitude_m <= 0:
            return None

        camera_vector = self._pixel_to_camera_vector(
            pixel_x=detection.center_x_px,
            pixel_y=detection.center_y_px,
            frame_width=frame_width,
            frame_height=frame_height,
            camera_model=camera_model,
        )
        nav_vector = self._apply_attitude(
            camera_vector,
            roll_deg=telemetry.roll_deg,
            pitch_deg=telemetry.pitch_deg,
            yaw_deg=telemetry.yaw_deg,
        )
        if abs(nav_vector[2]) <= 1e-6:
            return None

        scale = telemetry.altitude_m / nav_vector[2]
        return CameraRelativeCoordinate(
            x_m=nav_vector[0] * scale,
            y_m=nav_vector[1] * scale,
            z_m=telemetry.altitude_m,
        )

    def suggest_camera_model_for_planar_distance(self, detection: Detection, frame_shape, telemetry: TelemetrySample, target_planar_m: float):
        if target_planar_m <= 0 or telemetry.altitude_m <= 0:
            return None

        low = 0.1
        high = 2.0
        best_model = None
        best_error = None

        for _ in range(40):
            scale = (low + high) / 2.0
            candidate_model = CameraModel(
                horizontal_fov_deg=self.camera_model.horizontal_fov_deg * scale,
                vertical_fov_deg=self.camera_model.vertical_fov_deg * scale,
            )
            coord = self.project_with_camera_model(
                detection=detection,
                frame_shape=frame_shape,
                telemetry=telemetry,
                camera_model=candidate_model,
            )
            if coord is None:
                return None

            planar_m = math.hypot(coord.x_m, coord.y_m)
            error = planar_m - target_planar_m
            if best_error is None or abs(error) < abs(best_error):
                best_error = error
                best_model = candidate_model

            if error > 0:
                high = scale
            else:
                low = scale

        return best_model

    def _pixel_to_camera_vector(self, pixel_x, pixel_y, frame_width, frame_height, camera_model: CameraModel):
        delta_u_px = pixel_x - (frame_width / 2.0)
        delta_v_px = pixel_y - (frame_height / 2.0)

        theta_u = math.radians(
            delta_u_px * (camera_model.horizontal_fov_deg / frame_width)
        )
        theta_v = math.radians(
            delta_v_px * (camera_model.vertical_fov_deg / frame_height)
        )
        return (
            math.tan(theta_u),
            math.tan(theta_v),
            1.0,
        )

    def _apply_attitude(self, vector, roll_deg, pitch_deg, yaw_deg):
        roll = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw = math.radians(yaw_deg)
        x_cam, y_cam, z_cam = vector

        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        rotation = (
            (
                cos_pitch * cos_yaw,
                sin_roll * sin_pitch * cos_yaw - cos_roll * sin_yaw,
                cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw,
            ),
            (
                cos_pitch * sin_yaw,
                sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw,
                cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw,
            ),
            (
                -sin_pitch,
                sin_roll * cos_pitch,
                cos_roll * cos_pitch,
            ),
        )

        x_nav = (
            rotation[0][0] * x_cam
            + rotation[0][1] * y_cam
            + rotation[0][2] * z_cam
        )
        y_nav = (
            rotation[1][0] * x_cam
            + rotation[1][1] * y_cam
            + rotation[1][2] * z_cam
        )
        z_nav = (
            rotation[2][0] * x_cam
            + rotation[2][1] * y_cam
            + rotation[2][2] * z_cam
        )
        return (x_nav, y_nav, z_nav)
