#!/usr/bin/env python3
# Copyright 2026 Juan S. Cely G.
# Licensed under the Apache License, Version 2.0 (see LICENSE)
"""Frames (PNG) -> animated GIF for the documentation.

    make_gif.py <out.gif> <frame.png>... [--fps 10] [--width 400] [--colors 48] [--follow]
                [--zoom 2.2] [--speed 1.0] [--skip <s>]

--follow: crop each frame around the robot (the pixels that differ from the background, the
          crop center smoothed), for a fixed camera (Gazebo) and a robot that walks away.
--speed:  play faster (2.0: half the duration) by dropping frames.
"""
import argparse
import sys

import numpy as np
from PIL import Image


def background(files):
    """Median of frames spread over the clip: the scene without the (moving) robot"""
    pick = files[::max(1, len(files) // 40)]
    stack = np.stack([np.asarray(Image.open(f).convert('RGB'), dtype=np.uint8) for f in pick])
    return np.median(stack, axis=0).astype(np.int16)


def robot_box(im, bg):
    diff = np.abs(np.asarray(im, dtype=np.int16) - bg).max(axis=2) > 30
    ys, xs = np.nonzero(diff)
    if len(xs) < 50:
        return None
    # robust extent (outliers: shadows far away, noise)
    x0, x1 = np.percentile(xs, [1, 99])
    y0, y1 = np.percentile(ys, [1, 99])
    return x0, y0, x1, y1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('frames', nargs='+')
    ap.add_argument('--fps', type=float, default=10.0)
    ap.add_argument('--width', type=int, default=400)
    ap.add_argument('--colors', type=int, default=48)
    ap.add_argument('--follow', action='store_true')
    ap.add_argument('--zoom', type=float, default=2.2, help='crop width / robot height (--follow)')
    ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--skip', type=float, default=0.0, help='seconds dropped at the start')
    a = ap.parse_args()
    files = a.frames[int(a.skip * a.fps):][::max(1, int(round(a.speed)))]
    frames, cx, cy, size = [], None, None, None
    bg = background(files) if a.follow else None
    for f in files:
        im = Image.open(f).convert('RGB')
        W, H = im.size
        if a.follow:
            box = robot_box(im, bg)
            if box:
                bx, by = (box[0] + box[2]) / 2, (box[1] + box[3]) / 2
                s = max(box[3] - box[1], (box[2] - box[0]) * 9 / 16) * a.zoom
                cx = bx if cx is None else 0.85 * cx + 0.15 * bx
                cy = by if cy is None else 0.85 * cy + 0.15 * by
                size = s if size is None else 0.95 * size + 0.05 * s
            if cx is not None:
                h = min(size, H)
                w = min(h * 16 / 9, W)
                l = min(max(cx - w / 2, 0), W - w)
                t = min(max(cy - h / 2, 0), H - h)
                im = im.crop((int(l), int(t), int(l + w), int(t + h)))
        h = int(a.width * 9 / 16)
        frames.append(im.resize((a.width, h), Image.LANCZOS))
    if not frames:
        sys.exit('no frames')
    pal = [f.quantize(colors=a.colors, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE) for f in frames]
    pal[0].save(a.out, save_all=True, append_images=pal[1:], duration=int(1000 / a.fps), loop=0,
                optimize=True, disposal=1)
    print(f'{a.out}: {len(frames)} frames')


if __name__ == '__main__':
    main()
