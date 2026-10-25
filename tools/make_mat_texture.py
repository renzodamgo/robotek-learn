#!/usr/bin/env python3
"""Draw the game mat texture used by the simulation (our own drawing, no WRO artwork).

    python3 tools/make_mat_texture.py            # writes mat.png in robotek_gazebo's model folder

What it draws, in metres (the mat is 3.2 x 3.2 m; the image's top is +y and its right is +x, which is how Gazebo
lays the texture on the mat):

* a dark border and the 3.0 x 3.0 m white driving area;
* the dark square under the island;
* the eight coloured lines that tell the direction of the lap: from each corner of the island one blue line and one
  orange line run to the outer wall, 2 cm wide. The pattern repeats every 90 degrees. The positions follow the
  published field rules; only these numbers are kept here.

Needs numpy; writes the PNG with the standard library only.
"""
import argparse
import math
import struct
import zlib
from pathlib import Path

import numpy as np

MAT = 3.2                       # m, the mat
FIELD = 3.0                     # m, the driving area
BORDER = (25, 13, 65)           # RGB
WHITE = (255, 255, 255)
CENTRE = (36, 31, 88)           # the floor under the island
BLUE = (15, 77, 157)
ORANGE = (240, 128, 29)
LINE_WIDTH = 0.0195             # m
ISLAND = 0.5                    # m, half-size of the island (its corners are the start of every line)
BLUE_END = (1.5, 1.082)         # where the top-right blue line meets the outer wall
ORANGE_END = (1.082, 1.5)       # ... and the orange one; the other corners are this turned by 90 degrees


def rotate(point, quarter_turns):
    x, y = point
    for _ in range(quarter_turns % 4):
        x, y = -y, x
    return x, y


def line_segments():
    """[(colour, (x0, y0), (x1, y1))] for the eight lines."""
    segments = []
    for k in range(4):
        corner = rotate((ISLAND, ISLAND), k)
        segments.append((BLUE, corner, rotate(BLUE_END, k)))
        segments.append((ORANGE, corner, rotate(ORANGE_END, k)))
    return segments


def distance_to_segment(px, py, a, b):
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    t = np.clip(((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy), 0.0, 1.0)
    return np.hypot(px - (ax + t * dx), py - (ay + t * dy))


def render(size):
    pixel = MAT / size
    xs = (np.arange(size) + 0.5) * pixel - MAT / 2
    ys = MAT / 2 - (np.arange(size) + 0.5) * pixel
    px, py = np.meshgrid(xs, ys)
    image = np.zeros((size, size, 3), dtype=np.float64)
    image[:] = BORDER
    image[(np.abs(px) <= FIELD / 2) & (np.abs(py) <= FIELD / 2)] = WHITE
    image[(np.abs(px) <= ISLAND - 0.02) & (np.abs(py) <= ISLAND - 0.02)] = CENTRE
    for colour, a, b in line_segments():
        coverage = np.clip(0.5 + (LINE_WIDTH / 2 - distance_to_segment(px, py, a, b)) / pixel, 0.0, 1.0)
        coverage *= (np.abs(px) <= FIELD / 2) & (np.abs(py) <= FIELD / 2)       # lines stop at the outer wall
        image += coverage[..., None] * (np.array(colour, dtype=np.float64) - image)
    return np.clip(np.rint(image), 0, 255).astype(np.uint8)


def write_png(path, rgb):
    height, width, _ = rgb.shape
    raw = b''.join(b'\x00' + rgb[row].tobytes() for row in range(height))

    def chunk(tag, data):
        body = tag + data
        return struct.pack('>I', len(data)) + body + struct.pack('>I', zlib.crc32(body) & 0xFFFFFFFF)

    png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
           + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))
    Path(path).write_bytes(png)


def main():
    root = Path(__file__).resolve().parents[1]
    default = root / 'src/robotek_gazebo/models/robotek_game_mat/materials/textures/mat.png'
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--out', type=Path, default=default)
    parser.add_argument('--size', type=int, default=2048, help='pixels per side')
    args = parser.parse_args()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    write_png(args.out, render(args.size))
    print(f'wrote {args.out} ({args.size}x{args.size}, {args.out.stat().st_size / 1e3:.0f} kB)')


if __name__ == '__main__':
    main()
