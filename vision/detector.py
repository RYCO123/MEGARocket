import cv2

from .config import TARGET_COLOR_RANGES
from .models import Detection


class HSVRange:
    def __init__(self, lower, upper):
        self.lower = lower
        self.upper = upper


class ColorBlobDetector:
    def __init__(self, label, hsv_ranges, min_area_px=800):
        self.label = label
        self.hsv_ranges = hsv_ranges
        self.min_area_px = min_area_px

    def build_mask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = None
        for hsv_range in self.hsv_ranges:
            partial = cv2.inRange(hsv, hsv_range.lower, hsv_range.upper)
            mask = partial if mask is None else cv2.bitwise_or(mask, partial)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask

    def detect(self, frame):
        mask = self.build_mask(frame)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < self.min_area_px:
            return None

        x, y, width, height = cv2.boundingRect(contour)
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            center_x = x + (width / 2.0)
            center_y = y + (height / 2.0)
        else:
            center_x = moments["m10"] / moments["m00"]
            center_y = moments["m01"] / moments["m00"]

        frame_area = max(1.0, float(frame.shape[0] * frame.shape[1]))
        confidence = min(1.0, area / (0.15 * frame_area))
        return Detection(
            label=self.label,
            confidence=confidence,
            center_x_px=center_x,
            center_y_px=center_y,
            area_px=area,
            x_px=x,
            y_px=y,
            width_px=width,
            height_px=height,
        )


def build_detector(name, min_area_px=800):
    if name not in TARGET_COLOR_RANGES:
        supported = ", ".join(sorted(TARGET_COLOR_RANGES))
        raise ValueError(f"Unknown detector preset {name!r}. Choose from: {supported}")
    hsv_ranges = [HSVRange(lower, upper) for lower, upper in TARGET_COLOR_RANGES[name]]
    return ColorBlobDetector(name, hsv_ranges, min_area_px=min_area_px)
