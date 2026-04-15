import argparse
import csv
import math
import platform
import queue
import threading
import time
from pathlib import Path

from vision import (
    CameraModel,
    DEFAULT_CAMERA_MODEL,
    GroundProjector,
    StaticTelemetryProvider,
    TELEMETRY_SCENARIOS,
    VisionTracker,
)
from vision.config import DEFAULT_TARGET_COLOR, TARGET_COLOR_RANGES
from vision.config import DEFAULT_CAMERA_MODEL_NAME, DEFAULT_TARGET_MIN_AREA_PX
from vision.config import BENCH_TARGET_OFFSET_M

WINDOW_NAME = "Vision Tracker"
MASK_WINDOW_NAME = "Vision Mask"
CALIBRATION_SAMPLE_COUNT = 10
CALIBRATION_LOG_PATH = Path(__file__).resolve().parent / "vision_calibration_log.csv"
LAST_CLICK_INFO = {
    "pixel": None,
    "bgr": None,
    "hsv": None,
}


class FrameGrabber:
    def __init__(self, capture):
        self.capture = capture
        self.frame = None
        self.error = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        self._thread.join(timeout=1.0)

    def get_latest_frame(self):
        with self._lock:
            return self.frame

    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                ret, frame = self.capture.read()
            except Exception as exc:
                self.error = str(exc)
                break

            if not ret:
                time.sleep(0.02)
                continue

            with self._lock:
                self.frame = frame


class DistanceInputListener:
    def __init__(self):
        self.queue = queue.Queue()
        self._thread = threading.Thread(target=self._input_loop, daemon=True)

    def start(self):
        self._thread.start()

    def _input_loop(self):
        while True:
            try:
                raw = input(
                    "Enter center-to-marker distance in inches to log the next sample set: "
                ).strip()
            except EOFError:
                return
            except KeyboardInterrupt:
                return

            if not raw:
                continue

            try:
                distance_in = float(raw)
            except ValueError:
                print(f"Ignoring invalid distance input: {raw!r}")
                continue

            self.queue.put(distance_in)


class CalibrationLogger:
    def __init__(self, csv_path: Path, sample_count: int):
        self.csv_path = csv_path
        self.sample_count = sample_count
        self._pending_distance_in = None
        self._samples = []
        self._ensure_header()

    def queue_distance(self, distance_in: float):
        self._pending_distance_in = distance_in
        self._samples = []
        print(
            f"Queued calibration capture for {distance_in:.2f} in. "
            f"Collecting the next {self.sample_count} detections."
        )

    def active_target_distance_m(self):
        if self._pending_distance_in is None:
            return BENCH_TARGET_OFFSET_M
        return self._pending_distance_in * 0.0254

    def status_text(self):
        if self._pending_distance_in is None:
            return "calibration: waiting for distance input"
        return (
            f"calibration target={self._pending_distance_in:.2f} in "
            f"samples={len(self._samples)}/{self.sample_count}"
        )

    def maybe_collect(self, result):
        if self._pending_distance_in is None:
            return
        if result.detection is None or result.coordinate is None:
            return

        detection = result.detection
        coord = result.coordinate
        planar_m = math.hypot(coord.x_m, coord.y_m)
        self._samples.append(
            {
                "pixel_x": detection.center_x_px,
                "pixel_y": detection.center_y_px,
                "box_x": detection.x_px,
                "box_y": detection.y_px,
                "box_w": detection.width_px,
                "box_h": detection.height_px,
                "area_px": detection.area_px,
                "x_m": coord.x_m,
                "y_m": coord.y_m,
                "z_m": coord.z_m,
                "planar_m": planar_m,
            }
        )

        if len(self._samples) >= self.sample_count:
            self._flush_average(result)

    def _ensure_header(self):
        if self.csv_path.exists():
            return
        with self.csv_path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(
                [
                    "timestamp",
                    "target_distance_in",
                    "target_distance_m",
                    "sample_count",
                    "avg_pixel_x",
                    "avg_pixel_y",
                    "avg_box_x",
                    "avg_box_y",
                    "avg_box_w",
                    "avg_box_h",
                    "avg_area_px",
                    "avg_x_m",
                    "avg_y_m",
                    "avg_z_m",
                    "avg_planar_m",
                    "error_m",
                    "error_pct",
                    "camera_height_m",
                    "camera_hfov_deg",
                    "camera_vfov_deg",
                    "target_color",
                ]
            )

    def _flush_average(self, result):
        target_distance_in = self._pending_distance_in
        target_distance_m = target_distance_in * 0.0254
        sample_count = len(self._samples)

        def avg(key):
            return sum(sample[key] for sample in self._samples) / sample_count

        avg_planar_m = avg("planar_m")
        error_m = avg_planar_m - target_distance_m
        error_pct = (error_m / target_distance_m) * 100.0 if target_distance_m > 1e-9 else 0.0

        with self.csv_path.open("a", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(
                [
                    time.strftime("%Y-%m-%d %H:%M:%S"),
                    f"{target_distance_in:.3f}",
                    f"{target_distance_m:.6f}",
                    sample_count,
                    f"{avg('pixel_x'):.3f}",
                    f"{avg('pixel_y'):.3f}",
                    f"{avg('box_x'):.3f}",
                    f"{avg('box_y'):.3f}",
                    f"{avg('box_w'):.3f}",
                    f"{avg('box_h'):.3f}",
                    f"{avg('area_px'):.3f}",
                    f"{avg('x_m'):.6f}",
                    f"{avg('y_m'):.6f}",
                    f"{avg('z_m'):.6f}",
                    f"{avg_planar_m:.6f}",
                    f"{error_m:+.6f}",
                    f"{error_pct:+.3f}",
                    f"{result.telemetry.altitude_m:.6f}",
                    f"{result.camera_model.horizontal_fov_deg:.3f}",
                    f"{result.camera_model.vertical_fov_deg:.3f}",
                    result.detection.label,
                ]
            )

        print(
            "Logged calibration sample "
            f"target_in={target_distance_in:.2f} "
            f"planar_m={avg_planar_m:.3f} "
            f"error_m={error_m:+.3f} "
            f"error_pct={error_pct:+.1f} "
            f"csv={self.csv_path}"
        )
        self._pending_distance_in = None
        self._samples = []
def parse_args():
    parser = argparse.ArgumentParser(
        description="Open live video and estimate a ground target's camera-relative position."
    )
    parser.add_argument(
        '--source',
        default='0',
        help="Camera index or stream/device source to open.",
    )
    parser.add_argument(
        '--target-color',
        default=DEFAULT_TARGET_COLOR,
        choices=tuple(sorted(TARGET_COLOR_RANGES)),
        help="Color preset for the target blob detector.",
    )
    parser.add_argument(
        '--min-area',
        type=int,
        default=DEFAULT_TARGET_MIN_AREA_PX,
        help="Minimum contour area in pixels before a detection is accepted.",
    )
    parser.add_argument(
        '--scenario',
        default='bench_nadir',
        choices=tuple(sorted(TELEMETRY_SCENARIOS)),
        help="Fixed telemetry scenario used to imitate live vehicle inputs.",
    )
    parser.add_argument(
        '--altitude-m',
        type=float,
        help="Override scenario altitude in meters.",
    )
    parser.add_argument(
        '--pitch-deg',
        type=float,
        help="Override scenario pitch in degrees.",
    )
    parser.add_argument(
        '--roll-deg',
        type=float,
        help="Override scenario roll in degrees.",
    )
    parser.add_argument(
        '--yaw-deg',
        type=float,
        help="Override scenario yaw in degrees.",
    )
    parser.add_argument(
        '--hfov-deg',
        type=float,
        default=DEFAULT_CAMERA_MODEL.horizontal_fov_deg,
        help="Camera horizontal field of view in degrees.",
    )
    parser.add_argument(
        '--vfov-deg',
        type=float,
        default=DEFAULT_CAMERA_MODEL.vertical_fov_deg,
        help="Camera vertical field of view in degrees.",
    )
    parser.add_argument(
        '--calibration-input',
        action='store_true',
        help="Enable terminal input for logging calibration distances while the video feed runs.",
    )
    return parser.parse_args()


def normalize_source(source):
    try:
        return int(source)
    except ValueError:
        return source


def build_tracker(args):
    from vision.detector import build_detector

    detector = build_detector(args.target_color, min_area_px=args.min_area)

    if any(value is not None for value in (args.altitude_m, args.pitch_deg, args.roll_deg, args.yaw_deg)):
        base = TELEMETRY_SCENARIOS[args.scenario]
        telemetry = StaticTelemetryProvider.from_values(
            altitude_m=args.altitude_m if args.altitude_m is not None else base.altitude_m,
            pitch_deg=args.pitch_deg if args.pitch_deg is not None else base.pitch_deg,
            roll_deg=args.roll_deg if args.roll_deg is not None else base.roll_deg,
            yaw_deg=args.yaw_deg if args.yaw_deg is not None else base.yaw_deg,
        )
    else:
        telemetry = StaticTelemetryProvider.from_scenario(args.scenario)

    camera_model = CameraModel(
        horizontal_fov_deg=args.hfov_deg,
        vertical_fov_deg=args.vfov_deg,
    )
    projector = GroundProjector(camera_model)
    return VisionTracker(detector, telemetry, projector)


def open_video_capture(video_source):
    import cv2

    backend = cv2.CAP_ANY
    if isinstance(video_source, int) and platform.system() == "Darwin":
        backend = cv2.CAP_AVFOUNDATION

    cap = cv2.VideoCapture(video_source, backend)
    if hasattr(cv2, "CAP_PROP_BUFFERSIZE"):
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if hasattr(cv2, "CAP_PROP_OPEN_TIMEOUT_MSEC"):
        cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
    if hasattr(cv2, "CAP_PROP_READ_TIMEOUT_MSEC"):
        cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
    return cap


def make_status_frame(message, width=1280, height=720):
    import cv2
    import numpy as np

    frame = np.zeros((height, width, 3), dtype=np.uint8)
    frame[:] = (28, 28, 28)
    cv2.putText(
        frame,
        "Vision Tracker",
        (40, 80),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.3,
        (255, 255, 255),
        3,
    )
    cv2.putText(
        frame,
        message,
        (40, 150),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 220, 255),
        2,
    )
    cv2.putText(
        frame,
        "Press q to quit",
        (40, 200),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (200, 200, 200),
        2,
    )
    return frame


def handle_mouse_click(event, x, y, _flags, param):
    import cv2

    if event != cv2.EVENT_LBUTTONDOWN:
        return

    frame = param.get("frame")
    if frame is None:
        return

    if y < 0 or x < 0 or y >= frame.shape[0] or x >= frame.shape[1]:
        return

    bgr = frame[y, x]
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv = hsv_frame[y, x]
    LAST_CLICK_INFO["pixel"] = (x, y)
    LAST_CLICK_INFO["bgr"] = tuple(int(v) for v in bgr)
    LAST_CLICK_INFO["hsv"] = tuple(int(v) for v in hsv)
    print(
        "Clicked pixel "
        f"({x}, {y}) BGR={LAST_CLICK_INFO['bgr']} HSV={LAST_CLICK_INFO['hsv']}"
    )


def draw_overlay(frame, result, target_distance_m):
    import cv2

    height, _ = frame.shape[:2]
    detection = result.detection
    telemetry = result.telemetry

    cv2.putText(
        frame,
        f"bench z={telemetry.altitude_m:.2f}m yaw={telemetry.yaw_deg:.1f}",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2,
    )

    if detection is None:
        cv2.putText(
            frame,
            "target: not found",
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 0, 255),
            2,
        )
        return

    cv2.rectangle(
        frame,
        (detection.x_px, detection.y_px),
        (detection.x_px + detection.width_px, detection.y_px + detection.height_px),
        (0, 255, 0),
        2,
    )
    cv2.circle(
        frame,
        (int(detection.center_x_px), int(detection.center_y_px)),
        6,
        (0, 255, 255),
        -1,
    )

    cv2.putText(
        frame,
        (
            f"{detection.label} target px=({detection.center_x_px:.0f}, {detection.center_y_px:.0f})"
        ),
        (20, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 0),
        2,
    )

    if result.coordinate is None:
        cv2.putText(
            frame,
            "coord: unavailable",
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2,
        )
        return

    coord = result.coordinate
    cv2.putText(
        frame,
        f"relative xyz=({coord.x_m:.2f}, {coord.y_m:.2f}, {coord.z_m:.2f}) m",
        (20, 90),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 0),
        2,
    )
    planar_distance_m = math.hypot(coord.x_m, coord.y_m)
    planar_error_m = planar_distance_m - target_distance_m
    percent_error = 0.0
    if target_distance_m > 1e-9:
        percent_error = (planar_error_m / target_distance_m) * 100.0
    cv2.putText(
        frame,
        (
            f"planar={planar_distance_m:.2f}m "
            f"target={target_distance_m:.2f}m "
            f"error={planar_error_m:+.2f}m ({percent_error:+.1f}%)"
        ),
        (20, 120),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.60,
        (255, 220, 120),
        2,
    )
    cv2.putText(
        frame,
        "frame: x=right y=forward z=down",
        (20, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
    )


def draw_mask_overlay(mask, tracker):
    import cv2

    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    hsv_ranges = getattr(tracker.detector, "hsv_ranges", [])
    if hsv_ranges:
        range_text = " ".join(
            f"[{tuple(r.lower)} -> {tuple(r.upper)}]" for r in hsv_ranges
        )
        cv2.putText(
            mask_bgr,
            f"HSV ranges {range_text}",
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            2,
        )
    cv2.putText(
        mask_bgr,
        "White pixels are accepted by the detector",
        (20, 60),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        2,
    )
    if LAST_CLICK_INFO["pixel"] is not None:
        cv2.putText(
            mask_bgr,
            (
                f"last click px={LAST_CLICK_INFO['pixel']} "
                f"hsv={LAST_CLICK_INFO['hsv']}"
            ),
            (20, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 200, 200),
            2,
    )
    return mask_bgr

def main():
    args = parse_args()
    import cv2

    video_source = normalize_source(args.source)
    tracker = build_tracker(args)
    distance_listener = DistanceInputListener() if args.calibration_input else None
    calibration_logger = CalibrationLogger(
        csv_path=CALIBRATION_LOG_PATH,
        sample_count=CALIBRATION_SAMPLE_COUNT,
    )

    print(f"Opening video source {video_source!r}")
    print(
        "Tracking target with "
        f"color={args.target_color}, scenario={args.scenario}, "
        f"camera='{DEFAULT_CAMERA_MODEL_NAME}', "
        f"hfov={args.hfov_deg}, vfov={args.vfov_deg}"
    )
    print(f"Calibration CSV: {CALIBRATION_LOG_PATH}")
    if distance_listener is not None:
        distance_listener.start()
    else:
        print("Calibration terminal input is disabled for this run.")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.namedWindow(MASK_WINDOW_NAME, cv2.WINDOW_NORMAL)
    mouse_state = {"frame": None}
    cv2.setMouseCallback(WINDOW_NAME, handle_mouse_click, mouse_state)
    cv2.imshow(WINDOW_NAME, make_status_frame(f"Opening video source {video_source!r}"))
    cv2.imshow(MASK_WINDOW_NAME, make_status_frame("Waiting for mask frames"))
    cv2.waitKey(1)

    cap = open_video_capture(video_source)

    if not cap.isOpened():
        raise RuntimeError(f"Unable to open video source {video_source!r}")

    frame_grabber = FrameGrabber(cap)
    frame_grabber.start()
    start_time = time.time()
    try:
        while True:
            frame = frame_grabber.get_latest_frame()
            if frame is not None:
                if distance_listener is not None:
                    while True:
                        try:
                            distance_in = distance_listener.queue.get_nowait()
                        except queue.Empty:
                            break
                        calibration_logger.queue_distance(distance_in)
                mouse_state["frame"] = frame
                result = tracker.process_frame(frame)
                calibration_logger.maybe_collect(result)
                draw_overlay(frame, result, calibration_logger.active_target_distance_m())
                overlay_status = calibration_logger.status_text()
                cv2.putText(
                    frame,
                    overlay_status,
                    (20, 150),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (200, 255, 200),
                    2,
                )
                cv2.imshow(WINDOW_NAME, frame)
                mask = tracker.build_debug_mask(frame)
                if mask is not None:
                    cv2.imshow(MASK_WINDOW_NAME, draw_mask_overlay(mask, tracker))
            else:
                wait_s = time.time() - start_time
                status = f"Waiting for frames from source {video_source!r} ({wait_s:.1f}s)"
                if frame_grabber.error:
                    status = f"Camera read error: {frame_grabber.error}"
                cv2.imshow(WINDOW_NAME, make_status_frame(status))
                cv2.imshow(MASK_WINDOW_NAME, make_status_frame("Waiting for mask frames"))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        frame_grabber.stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
