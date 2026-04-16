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
        if detection is not None:
            coordinate = self.projector.project(detection, frame.shape, telemetry)
        return TrackingResult(
            detection=detection,
            coordinate=coordinate,
            telemetry=telemetry,
        )

    def build_debug_mask(self, frame):
        build_mask = getattr(self.detector, 'build_mask', None)
        if build_mask is None:
            return None
        return build_mask(frame)
