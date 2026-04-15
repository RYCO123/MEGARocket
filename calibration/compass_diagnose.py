"""
calibration/compass_diagnose.py

Diagnostic tool for compass calibration issues.

Streams raw magnetometer data while you rotate the vehicle and plots
the readings as a 3D scatter.  A healthy compass produces a sphere.
Soft-iron distortion squishes it into an ellipsoid.  Electrical
interference makes it noisy/scattered.

Run INSTEAD OF startup.py (connects directly to serial).

Usage:
    python3 calibration/compass_diagnose.py

Controls:
    Press Enter to stop collecting and show the plot.
"""

import sys
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
from pymavlink import mavutil

SERIAL_PORT       = '/dev/cu.usbserial-0001'
BAUD              = 57600
HEARTBEAT_TIMEOUT = 12
STREAM_RATE_HZ    = 10   # mag samples per second


# ---------------------------------------------------------------------------
# Connection
# ---------------------------------------------------------------------------

def connect():
    print(f"Connecting to {SERIAL_PORT} at {BAUD} baud ...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUD)
    hb = master.wait_heartbeat(timeout=HEARTBEAT_TIMEOUT)
    if hb is None:
        raise TimeoutError("No heartbeat. Is startup.py still running?")
    print(f"  Connected — system {master.target_system}, "
          f"component {master.target_component}\n")
    return master


MAG_MESSAGES = ('RAW_IMU', 'SCALED_IMU2', 'SCALED_IMU3')


def request_mag_stream(master):
    # Request RAW_SENS stream which includes all IMU/mag messages
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
        STREAM_RATE_HZ,
        1,  # start
    )


# ---------------------------------------------------------------------------
# Data collection
# ---------------------------------------------------------------------------

def collect(master, stop_event):
    """
    Collect mag samples from whichever IMU message carries live data.
    Detects which message type the external compass appears on by checking
    which one changes as the vehicle rotates.
    """
    # Accumulate per-source samples separately, then pick the live one
    buckets = {k: [] for k in MAG_MESSAGES}
    last_print = time.time()
    last_vals = {}

    while not stop_event.is_set():
        msg = master.recv_match(type=list(MAG_MESSAGES),
                                blocking=True, timeout=0.5)
        if msg is None:
            continue

        mtype = msg.get_type()
        x, y, z = msg.xmag, msg.ymag, msg.zmag

        # Only store if the value actually changed (skip frozen/duplicate)
        prev = last_vals.get(mtype)
        if prev != (x, y, z):
            buckets[mtype].append((x, y, z))
            last_vals[mtype] = (x, y, z)

        now = time.time()
        if now - last_print >= 1.0:
            counts = {k: len(v) for k, v in buckets.items() if v}
            live = max(counts, key=counts.get) if counts else 'none'
            if buckets[live] if live != 'none' else False:
                lx, ly, lz = buckets[live][-1]
                mag = np.sqrt(lx**2 + ly**2 + lz**2)
                print(f"  {sum(len(v) for v in buckets.values()):4d} samples  |  "
                      f"source={live}  "
                      f"x={lx:+6.0f}  y={ly:+6.0f}  z={lz:+6.0f}  |  "
                      f"|B|={mag:.0f} mG")
            last_print = now

    # Return samples from the source with the most unique readings
    best = max(MAG_MESSAGES, key=lambda k: len(buckets[k]))
    if not buckets[best]:
        return [], 'none'

    print(f"\n  Using source: {best} "
          f"({len(buckets[best])} unique samples)")
    for k, v in buckets.items():
        if v:
            print(f"    {k}: {len(v)} unique samples")

    return buckets[best], best


# ---------------------------------------------------------------------------
# Sphere fit (least squares)
# Fits: x² + y² + z² + Dx + Ey + Fz + G = 0
# ---------------------------------------------------------------------------

def fit_sphere(pts):
    pts = np.array(pts, dtype=float)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    A = np.column_stack([x, y, z, np.ones(len(pts))])
    b = -(x**2 + y**2 + z**2)

    result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    D, E, F, G = result

    cx = -D / 2
    cy = -E / 2
    cz = -F / 2
    radius = np.sqrt(cx**2 + cy**2 + cz**2 - G)

    # Residuals: distance from sphere surface for each point
    dist = np.sqrt((x - cx)**2 + (y - cy)**2 + (z - cz)**2)
    residuals = dist - radius

    return cx, cy, cz, radius, residuals


# ---------------------------------------------------------------------------
# Plot
# ---------------------------------------------------------------------------

def plot(samples, cx, cy, cz, radius, residuals):
    pts  = np.array(samples, dtype=float)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    fig = plt.figure(figsize=(13, 6))
    fig.suptitle("Compass Diagnostic", fontsize=14, fontweight='bold')

    # --- 3D scatter ---
    ax3d = fig.add_subplot(121, projection='3d')
    sc = ax3d.scatter(x, y, z, c=np.arange(len(x)), cmap='viridis',
                      s=4, alpha=0.6)
    ax3d.set_xlabel('X (mG)')
    ax3d.set_ylabel('Y (mG)')
    ax3d.set_zlabel('Z (mG)')
    ax3d.set_title('Raw mag data\n(should look like a sphere)')

    # Draw ideal sphere wireframe
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 20)
    sx = cx + radius * np.outer(np.cos(u), np.sin(v))
    sy = cy + radius * np.outer(np.sin(u), np.sin(v))
    sz = cz + radius * np.outer(np.ones(30), np.cos(v))
    ax3d.plot_surface(sx, sy, sz, alpha=0.08, color='cyan')

    # --- Residuals histogram ---
    ax2 = fig.add_subplot(122)
    ax2.hist(residuals, bins=40, color='steelblue', edgecolor='white')
    ax2.axvline(0, color='red', linewidth=1.5, label='ideal')
    ax2.set_xlabel('Residual (mG)')
    ax2.set_ylabel('Count')
    ax2.set_title('Sphere fit residuals\n(narrow = good, wide = distortion)')
    ax2.legend()

    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def report(samples, cx, cy, cz, radius, residuals):
    pts = np.array(samples, dtype=float)
    print()
    print("=" * 55)
    print("  COMPASS DIAGNOSTIC REPORT")
    print("=" * 55)
    print(f"  Samples collected : {len(samples)}")
    print(f"  Sphere center     : ({cx:+.1f}, {cy:+.1f}, {cz:+.1f}) mG")
    print(f"  Sphere radius     : {radius:.1f} mG")
    print(f"  Residual std dev  : {np.std(residuals):.1f} mG")
    print(f"  Residual max      : {np.max(np.abs(residuals)):.1f} mG")
    print()

    # Axis ranges — unequal ranges = soft iron distortion
    for label, col in zip(['X', 'Y', 'Z'], [0, 1, 2]):
        lo, hi = pts[:, col].min(), pts[:, col].max()
        print(f"  {label} range: {lo:+.0f} to {hi:+.0f}  "
              f"(span {hi - lo:.0f} mG)")

    std = np.std(residuals)
    print()
    if std < 15:
        print("  VERDICT: Good sphere fit. Calibration should work.")
        print("  If QGC still fails, try COMPASS_CAL_FIT = 32.")
    elif std < 40:
        print("  VERDICT: Moderate distortion. Likely soft-iron interference.")
        print("  Move the compass further from motors, ESCs, or battery wires.")
    else:
        print("  VERDICT: Severe distortion. Something magnetic is very close")
        print("  to the compass. Check for ferromagnetic mounting hardware,")
        print("  motor magnets, or high-current wiring near the GPS module.")
    print()

    # Axis span ratio — should be close to 1.0 for a sphere
    spans = [pts[:, i].max() - pts[:, i].min() for i in range(3)]
    ratio = max(spans) / min(spans) if min(spans) > 0 else 999
    if ratio > 1.3:
        print(f"  AXIS SPAN RATIO: {ratio:.2f}  (>1.3 = soft-iron squish)")
        labels = ['X', 'Y', 'Z']
        smallest = labels[np.argmin(spans)]
        print(f"  The {smallest}-axis is compressed — something is distorting")
        print(f"  the field along that axis.")
    print("=" * 55)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    print()
    print("=" * 55)
    print("  COMPASS DIAGNOSTIC")
    print("=" * 55)
    print("""
Rotate the vehicle slowly through ALL orientations:
  - nose up, nose down
  - each side down
  - level, inverted

Press Enter when done to see the plot and report.
""")

    master = connect()
    request_mag_stream(master)

    stop_event = threading.Event()

    print("Collecting... (press Enter to stop)\n")
    collector = threading.Thread(target=lambda: collect.__wrapped__(master, stop_event)
                                 if hasattr(collect, '__wrapped__') else None,
                                 daemon=True)

    # Run collector in thread, wait for Enter
    result = [None]

    def _collect():
        result[0] = collect(master, stop_event)

    t = threading.Thread(target=_collect, daemon=True)
    t.start()

    input()   # wait for Enter
    stop_event.set()
    t.join(timeout=2)

    samples, source = result[0] if result[0] else ([], 'none')

    if source == 'none' or len(samples) < 50:
        print()
        print("ERROR: No live magnetometer data found.")
        print("  All IMU messages returned static/frozen values.")
        print("  This means the compass chip is not updating.")
        print("  Check: I2C wiring, compass power, COMPASS_ENABLE=1")
        sys.exit(1)

    cx, cy, cz, radius, residuals = fit_sphere(samples)
    report(samples, cx, cy, cz, radius, residuals)
    plot(samples, cx, cy, cz, radius, residuals)


if __name__ == '__main__':
    main()
