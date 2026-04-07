import argparse
import cv2


def parse_args():
    parser = argparse.ArgumentParser(description="Open the configured video source.")
    parser.add_argument(
        '--source',
        default='0',
        help="Camera index or stream/device source to open.",
    )
    return parser.parse_args()


def normalize_source(source):
    try:
        return int(source)
    except ValueError:
        return source


args = parse_args()
video_source = normalize_source(args.source)

print(f"Opening video source {video_source!r}")

cap = cv2.VideoCapture(video_source)

if not cap.isOpened():
    raise RuntimeError(f"Unable to open video source {video_source!r}")

while True:
    ret, frame = cap.read()
    if ret:
        cv2.imshow("Drone Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
