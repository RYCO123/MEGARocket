"""
vision/analyze_benchmark.py

Recalculates error columns for all benchmark CSVs in vision/results/, then
generates analysis plots comparing flat (0°/0°) and tilted runs.

Plots produced:
  1. theta_vs_pixel.png         — calibration lookup curves (x and y axes)
  2. actual_vs_predicted.png    — scatter + best-fit line, flat run only
  3. error_vs_distance.png      — error vs distance, all runs overlaid
  4. position_map_<tag>.png     — 2-D position map per run
  5. attitude_comparison.png    — side-by-side error comparison across runs
                                  (only generated when >1 run exists)

Usage:
    python3 vision/analyze_benchmark.py
    python3 vision/analyze_benchmark.py --flat benchmark.csv --tilted benchmark_p5d0_r3d0_20260416.csv
"""

import argparse
import csv
import math
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

RESULTS_DIR = Path(__file__).resolve().parent / "results"
CAL_CSV     = RESULTS_DIR / "calibration.csv"

# Colours per run (flat always first)
PALETTE = ['steelblue', 'tomato', 'seagreen', 'darkorange', 'mediumpurple']


# ── CSV helpers ───────────────────────────────────────────────────────────────

def recalculate_and_load(path: Path) -> list[dict]:
    """Recompute err_* from pred_* and actual_*, write back, return rows."""
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))

    for r in rows:
        px = float(r['pred_x_m'])
        py = float(r['pred_y_m'])
        ax = float(r['actual_x_m'])
        ay = float(r['actual_y_m'])
        ex = px - ax
        ey = py - ay
        ed = math.hypot(ex, ey)
        ad = math.hypot(ax, ay)
        ep = (ed / ad * 100.0) if ad > 1e-4 else 0.0
        r['err_x_m']    = f"{ex:+.4f}"
        r['err_y_m']    = f"{ey:+.4f}"
        r['err_dist_m'] = f"{ed:.4f}"
        r['err_pct']    = f"{ep:+.1f}"

    fieldnames = list(rows[0].keys())
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    return rows


def arrays(rows):
    actual_x = np.array([float(r['actual_x_m'])  for r in rows])
    actual_y = np.array([float(r['actual_y_m'])  for r in rows])
    pred_x   = np.array([float(r['pred_x_m'])    for r in rows])
    pred_y   = np.array([float(r['pred_y_m'])    for r in rows])
    err_x    = np.array([float(r['err_x_m'])     for r in rows])
    err_y    = np.array([float(r['err_y_m'])     for r in rows])
    err_dist = np.array([float(r['err_dist_m'])  for r in rows])
    actual_d = np.hypot(actual_x, actual_y)
    samples  = [int(r['sample']) for r in rows]
    return actual_x, actual_y, pred_x, pred_y, err_x, err_y, err_dist, actual_d, samples


def run_label(rows) -> str:
    p = float(rows[0].get('pitch_deg', 0.0))
    r = float(rows[0].get('roll_deg',  0.0))
    if abs(p) < 0.05 and abs(r) < 0.05:
        return "Flat (0°/0°)"
    return f"Pitch {p:+.1f}°  Roll {r:+.1f}°"


def _polyfit(x, y, deg=1):
    coeffs = np.polyfit(x, y, deg)
    x_l = np.linspace(x.min(), x.max(), 200)
    y_l = np.polyval(coeffs, x_l)
    ss_res = np.sum((y - np.polyval(coeffs, x))**2)
    ss_tot = np.sum((y - y.mean())**2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return x_l, y_l, r2


# ── Plot 1: Theta vs Pixel ────────────────────────────────────────────────────

def plot_theta_vs_pixel(out):
    xs_px, xs_th, ys_px, ys_th = [], [], [], []
    with open(CAL_CSV, newline='', encoding='utf-8') as f:
        for row in csv.DictReader(f):
            px = float(row['pixel_offset_px'])
            th = float(row['theta_rad'])
            if row['axis'] == 'x':
                xs_px.append(px); xs_th.append(th)
            else:
                ys_px.append(px); ys_th.append(th)

    xs_px = np.array(xs_px); xs_th = np.array(xs_th)
    ys_px = np.array(ys_px); ys_th = np.array(ys_th)

    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(xs_px, np.degrees(xs_th), 'o-', color='steelblue', lw=2, ms=6,
            label='X axis (craft fwd / image horizontal)')
    ax.plot(ys_px, np.degrees(ys_th), 's--', color='tomato', lw=2, ms=6,
            label='Y axis (craft right / image vertical)')
    ax.set_xlabel('Pixel offset from centre (px)', fontsize=10)
    ax.set_ylabel('Theta (°)', fontsize=10)
    ax.set_title('Calibration: Theta vs Pixel Offset', fontsize=12, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.35)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved: {out}")


# ── Plot 2: Actual vs Predicted (flat run) ────────────────────────────────────

def plot_actual_vs_predicted(rows, out):
    actual_x, actual_y, pred_x, pred_y, err_x, err_y, *_ = arrays(rows)
    label = run_label(rows)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    for ax, act, pred, err, axis_label, color in [
        (axes[0], actual_x, pred_x, err_x, 'X  (right, m)', 'steelblue'),
        (axes[1], actual_y, pred_y, err_y, 'Y  (forward, m)', 'tomato'),
    ]:
        ax.errorbar(act, pred, yerr=np.abs(err), fmt='o', color=color,
                    ecolor='gray', elinewidth=1, capsize=4, capthick=1,
                    markersize=7, zorder=3, label='Samples')
        lim = max(np.abs(np.concatenate([act, pred]))) * 1.2
        ax.plot([-lim, lim], [-lim, lim], 'k--', lw=1, alpha=0.5, label='Perfect (1:1)')
        if len(act) >= 2:
            x_l, y_l, r2 = _polyfit(act, pred)
            ax.plot(x_l, y_l, color=color, lw=2, alpha=0.7,
                    label=f'Best fit  R²={r2:.3f}')
        ax.set_xlabel(f'Actual {axis_label}', fontsize=10)
        ax.set_ylabel(f'Predicted {axis_label}', fontsize=10)
        ax.set_title(f'Actual vs Predicted — {axis_label}', fontsize=11, fontweight='bold')
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.35)
        ax.set_aspect('equal', adjustable='datalim')

    fig.suptitle(label, fontsize=10, color='gray')
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved: {out}")


# ── Plot 3: Error vs Distance (all runs overlaid) ─────────────────────────────

def plot_error_vs_distance(all_runs: list[tuple[str, list]], out):
    fig, axes = plt.subplots(2, 1, figsize=(9, 8), sharex=True)

    for idx, (label, rows) in enumerate(all_runs):
        color = PALETTE[idx % len(PALETTE)]
        actual_x, actual_y, _, _, err_x, err_y, err_dist, actual_d, samples = arrays(rows)

        ax0 = axes[0]
        ax0.scatter(actual_d, err_x,    color=color, s=55, marker='o', zorder=3,
                    label=f'{label} — err_x')
        ax0.scatter(actual_d, err_y,    color=color, s=55, marker='s', zorder=3,
                    label=f'{label} — err_y', alpha=0.6)

        ax1 = axes[1]
        ax1.scatter(actual_d, err_dist, color=color, s=65, zorder=3, label=label)
        if len(actual_d) >= 2:
            x_l, y_l, r2 = _polyfit(actual_d, err_dist)
            ax1.plot(x_l, y_l, color=color, lw=2, alpha=0.6,
                     label=f'{label} trend R²={r2:.3f}')
        # Annotate sample numbers for first run only (avoids clutter)
        if idx == 0:
            for d, e, s in zip(actual_d, err_dist, samples):
                ax1.annotate(str(s), (d, e), textcoords='offset points',
                             xytext=(4, 4), fontsize=7, color='gray')

    axes[0].axhline(0, color='k', lw=0.8, alpha=0.4)
    axes[0].set_ylabel('Signed error (m)', fontsize=10)
    axes[0].set_title('Signed Component Errors vs Ground-Truth Distance', fontsize=11,
                       fontweight='bold')
    axes[0].legend(fontsize=7, ncol=2)
    axes[0].grid(True, alpha=0.35)

    axes[1].set_xlabel('Ground-truth distance from centre (m)', fontsize=10)
    axes[1].set_ylabel('Euclidean error (m)', fontsize=10)
    axes[1].set_title('Euclidean Error vs Ground-Truth Distance', fontsize=11,
                       fontweight='bold')
    axes[1].legend(fontsize=8)
    axes[1].set_ylim(bottom=0)
    axes[1].grid(True, alpha=0.35)

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved: {out}")


# ── Plot 4: 2-D Position Map ──────────────────────────────────────────────────

def plot_position_map(rows, out):
    actual_x, actual_y, pred_x, pred_y, *_, samples = arrays(rows)
    label = run_label(rows)

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.axhline(0, color='k', lw=0.7, alpha=0.4)
    ax.axvline(0, color='k', lw=0.7, alpha=0.4)

    for ax_, ay_, px_, py_, s in zip(actual_x, actual_y, pred_x, pred_y, samples):
        ax.annotate('', xy=(px_, py_), xytext=(ax_, ay_),
                    arrowprops=dict(arrowstyle='->', color='gray', lw=1.2))
        ax.plot(ax_, ay_, 'o', color='steelblue', ms=9, zorder=4)
        ax.plot(px_, py_, 'x', color='tomato',    ms=9, mew=2, zorder=4)
        ax.annotate(str(s), ((ax_ + px_) / 2 + 0.012, (ay_ + py_) / 2 + 0.012),
                    fontsize=7, color='gray')

    from matplotlib.lines import Line2D
    ax.legend(handles=[
        Line2D([0], [0], marker='o', color='w', markerfacecolor='steelblue',
               markersize=9, label='Actual'),
        Line2D([0], [0], marker='x', color='tomato', markersize=9,
               markeredgewidth=2, label='Predicted', linestyle='None'),
    ], fontsize=9)

    ax.set_xlabel('X — craft right (m)', fontsize=10)
    ax.set_ylabel('Y — craft forward (m)', fontsize=10)
    ax.set_title(f'2-D Position Map — {label}\n(arrows: actual → predicted)',
                 fontsize=11, fontweight='bold')
    ax.set_aspect('equal', adjustable='datalim')
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved: {out}")


# ── Plot 5: Attitude Comparison (bar chart + scatter) ────────────────────────

def plot_attitude_comparison(all_runs: list[tuple[str, list]], out):
    labels   = [label for label, _ in all_runs]
    means    = []
    rmses    = []
    maxerrs  = []

    for _, rows in all_runs:
        *_, err_dist, _, _ = arrays(rows)
        means.append(np.mean(err_dist) * 100)
        rmses.append(math.sqrt(np.mean(err_dist**2)) * 100)
        maxerrs.append(np.max(err_dist) * 100)

    x = np.arange(len(labels))
    w = 0.25

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))

    # Left: grouped bar chart of summary stats
    ax = axes[0]
    ax.bar(x - w, means,   w, label='Mean error',  color='steelblue', alpha=0.85)
    ax.bar(x,     rmses,   w, label='RMSE',         color='tomato',    alpha=0.85)
    ax.bar(x + w, maxerrs, w, label='Max error',    color='seagreen',  alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=9)
    ax.set_ylabel('Error (cm)', fontsize=10)
    ax.set_title('Error Summary by Attitude', fontsize=11, fontweight='bold')
    ax.legend(fontsize=9)
    ax.grid(True, axis='y', alpha=0.35)

    # Right: per-sample error scatter, all runs overlaid
    ax2 = axes[1]
    for idx, (label, rows) in enumerate(all_runs):
        color = PALETTE[idx % len(PALETTE)]
        *_, err_dist, actual_d, samples = arrays(rows)
        ax2.scatter(actual_d, err_dist * 100, color=color, s=60,
                    label=label, zorder=3)
        if len(actual_d) >= 2:
            x_l, y_l, _ = _polyfit(actual_d, err_dist * 100)
            ax2.plot(x_l, y_l, color=color, lw=2, alpha=0.55)

    ax2.set_xlabel('Ground-truth distance from centre (m)', fontsize=10)
    ax2.set_ylabel('Euclidean error (cm)', fontsize=10)
    ax2.set_title('Per-Sample Error: Flat vs Tilted', fontsize=11, fontweight='bold')
    ax2.set_ylim(bottom=0)
    ax2.legend(fontsize=9)
    ax2.grid(True, alpha=0.35)

    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"Saved: {out}")


# ── Discover benchmark CSVs ───────────────────────────────────────────────────

def find_benchmarks(flat_path=None, tilted_paths=None):
    """Return list of (label, rows) sorted flat-first."""
    if flat_path:
        candidates = [Path(flat_path)]
        if tilted_paths:
            candidates += [Path(p) for p in tilted_paths]
    else:
        candidates = sorted(RESULTS_DIR.glob("benchmark*.csv"))

    runs = []
    for p in candidates:
        rows = recalculate_and_load(p)
        label = run_label(rows)
        runs.append((label, rows))
        print(f"Loaded: {p.name}  →  {label}  ({len(rows)} samples)")

    # Flat run first
    runs.sort(key=lambda x: (0 if x[0].startswith('Flat') else 1))
    return runs


# ── Summary printout ──────────────────────────────────────────────────────────

def print_summary(all_runs):
    print(f"\n{'─'*55}")
    for label, rows in all_runs:
        *_, err_dist, _, _ = arrays(rows)
        rmse = math.sqrt(np.mean(err_dist**2))
        print(f" {label}")
        print(f"   Mean : {np.mean(err_dist)*100:.1f} cm  |  "
              f"RMSE : {rmse*100:.1f} cm  |  "
              f"Max : {np.max(err_dist)*100:.1f} cm")
    print(f"{'─'*55}\n")


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Benchmark analysis and plots.")
    parser.add_argument('--flat',   help='Path to flat (0°/0°) benchmark CSV')
    parser.add_argument('--tilted', nargs='+', help='Path(s) to tilted benchmark CSV(s)')
    args = parser.parse_args()

    all_runs = find_benchmarks(args.flat, args.tilted)
    if not all_runs:
        print("No benchmark CSVs found in vision/results/")
        return

    plot_theta_vs_pixel(RESULTS_DIR / "theta_vs_pixel.png")

    # Actual vs predicted — flat run only (or first run)
    plot_actual_vs_predicted(all_runs[0][1], RESULTS_DIR / "actual_vs_predicted.png")

    plot_error_vs_distance(all_runs, RESULTS_DIR / "error_vs_distance.png")

    for label, rows in all_runs:
        tag = label.replace(' ', '_').replace('/', '').replace('°', 'deg').replace('+', 'p').replace('-', 'n')
        plot_position_map(rows, RESULTS_DIR / f"position_map_{tag}.png")

    if len(all_runs) > 1:
        plot_attitude_comparison(all_runs, RESULTS_DIR / "attitude_comparison.png")

    print_summary(all_runs)


if __name__ == '__main__':
    main()
