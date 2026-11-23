#!/usr/bin/env python3
"""Write sim/field.jpg, the live view's farmland ground (SWARM_DEMO_GROUND=field): a patchwork of
crop parcels, hedges, a dirt track, and a calm meadow where the 300 m field is. North is up;
2048 px over the 2000 m ground plane centred on the field. The same picture on every run.

    python3 tools/field_texture.py
"""
import os
import sys

import numpy as np
from PIL import Image, ImageFilter

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from sim import demo_world

N, SIZE_M = 2048, 2000.0
CROPS = [(74, 96, 52), (62, 84, 48), (88, 98, 56), (98, 100, 60), (70, 90, 62), (58, 78, 44)]
TILLED = [(116, 100, 74), (104, 90, 68), (124, 110, 82)]
MEADOW, HEDGE, TRACK = (72, 90, 56), (34, 48, 30), (138, 124, 96)
rng = np.random.default_rng(11)


def smooth(cells: int) -> np.ndarray:
    """Random values on a cells x cells grid, blown up smoothly to the picture: -1..1."""
    small = Image.fromarray(rng.integers(0, 256, (cells, cells), dtype=np.uint8))
    return np.asarray(small.resize((N, N), Image.BICUBIC), dtype=np.float32) / 127.5 - 1.0


def blur(mask: np.ndarray, radius: float) -> np.ndarray:
    picture = Image.fromarray((mask * 255).astype(np.uint8))
    return np.asarray(picture.filter(ImageFilter.GaussianBlur(radius)), dtype=np.float32) / 255.0


metre = (np.arange(N) + 0.5) * SIZE_M / N - SIZE_M / 2 + 150.0
x, y = np.meshgrid(metre, metre[::-1])                     # world metres, north up
# parcels: each pixel belongs to the nearest of 150 seeds, by the larger of |dx| and |dy| in a
# frame turned 12 deg and bent a little, so the edges are straight-ish but not a grid
u = 0.978 * x + 0.208 * y + 25.0 * smooth(6)
v = -0.208 * x + 0.978 * y + 25.0 * smooth(6)
seeds = rng.uniform(-900.0, 1200.0, (150, 2))
nearest, parcel = np.full((N, N), np.inf, dtype=np.float32), np.zeros((N, N), dtype=np.int16)
for k, (su, sv) in enumerate(seeds):
    d = np.maximum(np.abs(u - su), np.abs(v - sv) * 1.6)
    parcel[d < nearest], nearest = k, np.minimum(nearest, d)
on_field = (np.abs(x - 150.0) < 163.0) & (np.abs(y - 150.0) < 163.0)   # the field and its margin
parcel[on_field] = len(seeds)

tilled = rng.random(len(seeds) + 1) < 0.2
tone = np.array([(TILLED if t else CROPS)[rng.integers(3 if t else 6)] for t in tilled], np.float32)
tone[-1], tilled[-1] = MEADOW, False
angle = rng.uniform(0.0, np.pi, len(seeds) + 1)[parcel]    # row direction of each parcel
rows = np.sin((x * np.cos(angle) + y * np.sin(angle)) * 2 * np.pi / 7.0)
rows *= np.where(tilled[parcel], 6.0, 2.5) * ~on_field     # faint rows, none on the meadow
shade = rows + np.where(on_field, 3.0, 9.0) * smooth(24) + 4.0 * smooth(160)
rgb = tone[parcel] + shade[..., None]

edge = np.zeros((N, N), dtype=bool)                        # parcel edges
edge[:, 1:] |= parcel[:, 1:] != parcel[:, :-1]
edge[1:, :] |= parcel[1:, :] != parcel[:-1, :]
edge &= ~((np.abs(x - 150.0) < 166.0) & (np.abs(y - 150.0) < 166.0))   # no line round the meadow
hedged = (rng.random(len(seeds) + 1) < 0.45)[parcel] & ~on_field
hedge = np.clip(blur(edge & hedged, 2.5) * 4.0, 0.0, 1.0)          # dark hedges on some
margin = np.clip(blur(edge & ~hedged, 1.2) * 2.0, 0.0, 0.35)        # a pale margin on the rest
track = np.abs(y + 45.0 - 14.0 * np.sin(x / 260.0)) < 2.6          # one track south of the field,
track |= (np.abs(x - 452.0 - 12.0 * np.sin(y / 190.0)) < 2.2) & (y > -45.0)   # one going north
track = blur(track, 1.0)
under = np.zeros((N, N), dtype=bool)                       # dark undergrowth below each tree
for _, tx, ty, _, _ in demo_world.trees():
    under |= (x - tx) ** 2 + (y - ty) ** 2 < 6.0 ** 2
hedge = np.maximum(hedge, blur(under, 3.0))
for mask, colour in ((margin, TRACK), (hedge, HEDGE), (track, TRACK)):
    rgb = rgb * (1.0 - mask[..., None]) + (np.array(colour) + 5.0 * smooth(300)[..., None]) * mask[..., None]
rgb += rng.normal(0.0, 2.0, (N, N, 1))                     # grain
Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8)).save(demo_world.FIELD_TEXTURE, quality=82)
