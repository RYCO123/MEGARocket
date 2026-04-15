from .models import TrackingResult


class VisionTracker:
    def __init__(self, detector, telemetry_provider, projector):
        self.detector = detector
        self.telemetry_provider = telemetry_provider
        self.projector = projector

    def process_frame(self, frame):
        telemetry = self.telemetry_provider.get_latest()
        detection = self.detector.detect(frame)
        coordinate = None
        suggested_camera_model = None
        if detection is not None:
            coordinate = self.projector.project(detection, frame.shape, telemetry)
            try:
                from .config import BENCH_TARGET_OFFSET_M

                suggested_camera_model = self.projector.suggest_camera_model_for_planar_distance(
                    detection=detection,
                    frame_shape=frame.shape,
                    telemetry=telemetry,
                    target_planar_m=BENCH_TARGET_OFFSET_M,
                )
            except Exception:
                suggested_camera_model = None
        return TrackingResult(
            detection=detection,
            coordinate=coordinate,
            telemetry=telemetry,
            camera_model=self.projector.camera_model,
            suggested_camera_model=suggested_camera_model,
        )

    def build_debug_mask(self, frame):
        build_mask = getattr(self.detector, "build_mask", None)
        if build_mask is None:
            return None
        return build_mask(frame)
