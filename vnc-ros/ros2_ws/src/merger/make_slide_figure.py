#!/usr/bin/env python3
"""
make_slide_figure.py
--------------------
Build a single composite "before / after" figure suitable for the
intermediate-update slide deck. Inputs/outputs are written to ./results/.
"""

import os
import numpy as np
from PIL import Image, ImageDraw, ImageFont

RESULTS_DIR = "results"

PANEL_TITLES = [
    "Robot A's local map\n(unknown frame A)",
    "Robot B's local map\n(unknown frame B, rotated and translated)",
    "Alignment overlay\n(A green, B magenta, overlap dark)",
    "Merged global map\n(after ORB + RANSAC alignment)",
]
PANEL_FILES = [
    "01_input_grid_a.png",
    "02_input_grid_b.png",
    "04_alignment_overlay.png",
    "05_merged_map.png",
]


def load(path):
    return Image.open(os.path.join(RESULTS_DIR, path)).convert("RGB")


def main():
    imgs = [load(f) for f in PANEL_FILES]

    # Resize all panels to the same height to make a clean 2x2 grid.
    target_h = 300
    panels = []
    for im in imgs:
        ratio = target_h / im.height
        new_w = int(im.width * ratio)
        panels.append(im.resize((new_w, target_h), Image.NEAREST))

    # Per-row layout: two panels side by side, with a caption below each.
    caption_h = 60
    pad = 20
    title_h = 50

    # Figure out a uniform panel width to keep things tidy.
    panel_w = max(p.width for p in panels)
    row_w = 2 * panel_w + 3 * pad
    row_h = target_h + caption_h + pad

    fig_w = row_w
    fig_h = title_h + 2 * row_h + pad

    fig = Image.new("RGB", (fig_w, fig_h), (255, 255, 255))
    draw = ImageDraw.Draw(fig)

    try:
        title_font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
        caption_font = ImageFont.truetype(
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 14)
    except OSError:
        title_font = ImageFont.load_default()
        caption_font = ImageFont.load_default()

    draw.text(
        (pad, pad // 2),
        "Map Coordination Pipeline: Align and Fuse Two Robot Maps",
        fill=(0, 0, 0), font=title_font,
    )

    for i, (im, title) in enumerate(zip(panels, PANEL_TITLES)):
        row = i // 2
        col = i % 2
        x = pad + col * (panel_w + pad)
        y = title_h + row * row_h

        # Center the panel inside the cell.
        x_offset = (panel_w - im.width) // 2
        fig.paste(im, (x + x_offset, y))

        # Caption beneath the panel.
        for k, line in enumerate(title.split("\n")):
            draw.text(
                (x + 5, y + target_h + 5 + k * 18),
                line, fill=(0, 0, 0), font=caption_font,
            )

    out = os.path.join(RESULTS_DIR, "slide_composite.png")
    fig.save(out)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
