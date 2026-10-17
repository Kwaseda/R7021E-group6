#!/usr/bin/env python3
"""Lab 1 block diagram. Writes lab1-block-diagram.svg and .png.

Two simple closed loops, one per controller, drawn in the same visual language
as project-block-diagram-v2.svg: white ground, near-black 1.2 px outlines,
plain rectangles for what the course provides, a pastel fill for the block we
wrote, white chips on the arrow labels.
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch

W, H = 820, 470
INK = '#1a1a1a'
PX = 0.72                      # 1 px at 100 dpi, expressed in points
BLUE = ('#dae8fc', '#6c8ebf')
RED = ('#f8cecc', '#b85450')

fig = plt.figure(figsize=(W / 100, H / 100), dpi=100)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, W)
ax.set_ylim(H, 0)
ax.axis('off')
ax.add_patch(Rectangle((0, 0), W, H, fc='white', ec='none'))


def box(x, y, w, h, title, sub=None, fill='#ffffff', edge=INK, lw=1.2):
    ax.add_patch(Rectangle((x, y), w, h, fc=fill, ec=edge, lw=lw * PX))
    cx = x + w / 2
    ty = y + h / 2 - (9 if sub else 0)
    ax.text(cx, ty, title, ha='center', va='center', color=INK, fontsize=12.5 * PX)
    if sub:
        ax.text(cx, ty + 18, sub, ha='center', va='center', color=INK, fontsize=10.5 * PX)


def chip(x, y, text, size=10):
    ax.text(x, y, text, ha='center', va='center', color=INK, fontsize=size * PX,
            bbox=dict(boxstyle='square,pad=0.30', fc='white', ec='none'))


def arrow(pts):
    for (x0, y0), (x1, y1) in zip(pts, pts[1:-1]):
        ax.plot([x0, x1], [y0, y1], color=INK, lw=1.3 * PX, solid_capstyle='butt')
    (xa, ya), (xb, yb) = pts[-2], pts[-1]
    ax.add_patch(FancyArrowPatch((xa, ya), (xb, yb), arrowstyle='-|>',
                                 mutation_scale=7, color=INK, lw=1.3 * PX,
                                 shrinkA=0, shrinkB=0))


ax.add_patch(Rectangle((235, 6), 350, 24, fc='white', ec=INK, lw=1.2 * PX))
ax.text(410, 18, 'Lab 1: the two control loops', ha='center', va='center',
        color=INK, fontsize=13 * PX)

# ---- position control -------------------------------------------------
ax.text(40, 62, 'Position controller, tasks 1 and 2', ha='left', va='center',
        color=INK, fontsize=11.5 * PX)

box(190, 86, 230, 64, 'Position controller', 'NID, proportional, 20 Hz',
    fill=BLUE[0], edge=BLUE[1], lw=1.3)
box(540, 86, 230, 64, 'TurtleBot3 Burger', 'wheel control and motors')

arrow([(50, 118), (182, 118)])
chip(116, 118, '/new_position')
ax.text(116, 98, 'desired position', ha='center', va='center', color=INK, fontsize=10 * PX)

arrow([(420, 118), (532, 118)])
chip(476, 118, '/cmd_vel')
ax.text(476, 98, 'v, omega', ha='center', va='center', color=INK, fontsize=10 * PX)

arrow([(655, 150), (655, 192), (305, 192), (305, 158)])
chip(480, 192, '/odom')

# ---- wall following ---------------------------------------------------
ax.text(40, 272, 'Wall follower, task 4', ha='left', va='center',
        color=INK, fontsize=11.5 * PX)

box(190, 296, 230, 64, 'Wall follower', 'proportional on distance, 20 Hz',
    fill=RED[0], edge=RED[1], lw=1.3)
box(540, 296, 230, 64, 'TurtleBot3 Burger', 'wheel control and motors')

arrow([(420, 328), (532, 328)])
chip(476, 328, '/cmd_vel')
ax.text(476, 308, 'v, omega', ha='center', va='center', color=INK, fontsize=10 * PX)

arrow([(655, 360), (655, 402), (305, 402), (305, 368)])
chip(480, 402, '/scan')
ax.text(116, 328, 'no goal, reactive', ha='center', va='center',
        color=INK, fontsize=10 * PX)

out = '/home/dominiques-laptop/R7021E-group6/lab1/plots/lab1-block-diagram'
fig.savefig(out + '.png', dpi=200, facecolor='white')
fig.savefig(out + '.svg', facecolor='white')
print('wrote', out + '.png', 'and', out + '.svg')
