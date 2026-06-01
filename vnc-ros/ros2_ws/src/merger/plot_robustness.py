#!/usr/bin/env python3
"""
plot_robustness.py
------------------
Read the robustness-sweep CSV and produce a single PNG plot for the
intermediate-update slide: angle error and confidence as functions of
ground-truth rotation, with the success/failure threshold drawn in.
"""

import csv
import os
import matplotlib
matplotlib.use('Agg')  # no display
import matplotlib.pyplot as plt

RESULTS_DIR = 'results'
CSV_PATH = os.path.join(RESULTS_DIR, 'robustness_sweep.csv')
OUT_PATH = os.path.join(RESULTS_DIR, 'robustness_plot.png')


def main():
    rows = []
    with open(CSV_PATH) as fh:
        reader = csv.DictReader(fh)
        for r in reader:
            rows.append(r)

    thetas = [float(r['theta_gt_deg']) for r in rows]
    err = [float(r['angle_error_deg']) if r['angle_error_deg'] != 'nan'
           else None for r in rows]
    conf = [float(r['confidence']) for r in rows]
    success = [r['success'] == 'True' for r in rows]

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    fig.suptitle('Map Coordinator Robustness vs. Ground-Truth Rotation',
                 fontsize=13, fontweight='bold')

    # Top: angle error (log scale because the dynamic range is huge).
    err_plot = [(t, e) for t, e in zip(thetas, err) if e is not None]
    if err_plot:
        ts, es = zip(*err_plot)
        # Color: green for success, red for failure.
        colors = ['#2a9d2a' if s else '#cc3333'
                  for s, e in zip(success, err) if e is not None]
        ax1.bar(ts, es, color=colors, width=2.5, edgecolor='black',
                linewidth=0.5)
    ax1.set_yscale('symlog', linthresh=0.5)
    ax1.set_ylabel('Recovered angle error (deg, log)')
    ax1.axhline(1.0, color='gray', linestyle='--', linewidth=0.7,
                label='1 deg')
    ax1.grid(True, axis='y', alpha=0.3)
    ax1.legend(loc='upper left', fontsize=9)

    # Bottom: confidence.
    colors_conf = ['#2a9d2a' if s else '#cc3333' for s in success]
    ax2.bar(thetas, conf, color=colors_conf, width=2.5,
            edgecolor='black', linewidth=0.5)
    ax2.axhline(0.5, color='black', linestyle='--', linewidth=0.8,
                label='confidence threshold')
    ax2.set_xlabel('Ground-truth B->A rotation (deg)')
    ax2.set_ylabel('Confidence score')
    ax2.set_ylim(0, 1)
    ax2.grid(True, axis='y', alpha=0.3)
    ax2.legend(loc='upper right', fontsize=9)

    # Custom legend for color.
    from matplotlib.patches import Patch
    legend_elems = [
        Patch(facecolor='#2a9d2a', edgecolor='black', label='accepted'),
        Patch(facecolor='#cc3333', edgecolor='black', label='rejected'),
    ]
    ax1.legend(handles=legend_elems + [
        plt.Line2D([0], [0], color='gray', linestyle='--', label='1 deg'),
    ], loc='upper left', fontsize=9)

    plt.tight_layout()
    plt.savefig(OUT_PATH, dpi=150, bbox_inches='tight')
    print(f'wrote {OUT_PATH}')


if __name__ == '__main__':
    main()
