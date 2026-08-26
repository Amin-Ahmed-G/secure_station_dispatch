#!/usr/bin/env python3
"""
render_facility_map.py
──────────────────────
Renders a plan view of the facility with the planned tour drawn on top, from
``config/station_registry.yaml`` alone.

Two figures are produced:

  docs/facility_map.png   the shipped layout, with obstacle inflation haloes,
                          the four berths, marker sightlines and the tour path
                          as actually simulated by the control law
  docs/legacy_failure.png the original layout, showing the Assembly → Storage
                          leg driving through the Assembly marker panel

Usage:
    python3 scripts/render_facility_map.py
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.patches import Circle, FancyArrow, Rectangle  # noqa: E402
import yaml                                          # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)

from secure_station_dispatch.navigation import (      # noqa: E402
    ControlParams, DiffDriveController, PathPlanner, Rect,
    normalize_angle, obstacles_from_registry,
)

ACCENTS = {0: '#1f6fb2', 1: '#b56a14', 2: '#2f7d4f', 3: '#6b3fa0'}
INK = '#1d2126'
MUTED = '#8b949e'
HALO = '#c8d2dc'
STRUCTURE = '#4b5560'
PATH = '#d0342c'


def simulate_tour(registry, planner):
    """Replay the control law so the drawn path is the one the robot drives."""
    controller = DiffDriveController(ControlParams.from_registry(registry['navigation']))
    home = registry['world']['home_pose']
    x, y, yaw = home[0], home[1], home[2] if len(home) > 2 else 0.0
    trace = [(x, y)]
    dt = 0.05

    def run_to(target):
        nonlocal x, y, yaw
        for _ in range(2000):
            cmd = controller.command(x, y, yaw, target)
            if cmd.arrived:
                return
            yaw = normalize_angle(yaw + cmd.angular * dt)
            x += cmd.linear * math.cos(yaw) * dt
            y += cmd.linear * math.sin(yaw) * dt
            trace.append((x, y))

    for _marker_id, station in sorted(registry['stations'].items()):
        berth = tuple(station['expected_pose'][:2])
        for waypoint in planner.plan((x, y), berth).waypoints:
            run_to(waypoint)
    if registry['navigation'].get('return_home', True):
        for waypoint in planner.plan((x, y), (home[0], home[1])).waypoints:
            run_to(waypoint)
    return trace


def draw_rect(ax, rect: Rect, *, facecolor, edgecolor, alpha=1.0, lw=0.0, z=1,
              hatch=None):
    ax.add_patch(Rectangle((rect.min_x, rect.min_y), rect.dx, rect.dy,
                           facecolor=facecolor, edgecolor=edgecolor, alpha=alpha,
                           linewidth=lw, zorder=z, hatch=hatch))


def render_current(registry, out_path):
    planner = PathPlanner(
        obstacles=obstacles_from_registry(registry),
        inflation=registry['navigation']['robot_radius_m'] + registry['navigation']['safety_margin_m'],
        hard_inflation=registry['navigation']['robot_radius_m'],
        bounds=registry['world']['bounds'],
    )
    bounds = registry['world']['bounds']
    board = registry['world']['marker_board']

    fig, ax = plt.subplots(figsize=(11, 11.6))
    fig.patch.set_facecolor('white')

    ax.add_patch(Rectangle((bounds['x_min'], bounds['y_min']),
                           bounds['x_max'] - bounds['x_min'],
                           bounds['y_max'] - bounds['y_min'],
                           facecolor='#f4f5f6', edgecolor='none', zorder=0))

    for rect in planner.obstacles:
        draw_rect(ax, rect.inflated(planner.inflation), facecolor=HALO,
                  edgecolor='none', alpha=0.45, z=1)
    for rect in planner.obstacles:
        colour = ACCENTS.get(int(rect.name[-1]), STRUCTURE) if rect.name.startswith('bay_') else STRUCTURE
        draw_rect(ax, rect, facecolor=colour, edgecolor='none', z=3)

    # Marker sightlines: berth → board, with the detection cone.
    for marker_id, station in sorted(registry['stations'].items()):
        bx, by = station['expected_pose'][:2]
        mx, my = station['marker_pose'][:2]
        accent = ACCENTS[marker_id]
        ax.plot([bx, mx], [by, my], color=accent, lw=1.2, ls=(0, (4, 3)), zorder=4)
        ax.add_patch(Circle((bx, by), registry['navigation']['goal_tolerance_m'],
                            facecolor=accent, edgecolor='none', alpha=0.9, zorder=6))
        ax.add_patch(Circle((bx, by), station['pose_tolerance_m'], facecolor='none',
                            edgecolor=accent, lw=0.9, ls=':', alpha=0.7, zorder=5))
        yaw = station['approach_yaw']
        ax.add_patch(FancyArrow(bx, by, 0.42 * math.cos(yaw), 0.42 * math.sin(yaw),
                                width=0.035, head_width=0.16, head_length=0.16,
                                color=accent, zorder=7, length_includes_head=True))
        # Label inboard of the board and offset along it, so it never lands on
        # the sightline or outside the drawn facility.
        nx, ny = station['marker_normal']
        px, py = -ny, nx
        ax.text(mx + 0.45 * nx + 0.78 * px, my + 0.45 * ny + 0.78 * py,
                f'{station["station_name"].replace(" Station", "")}\nID {marker_id}',
                color=accent, fontsize=9.5, fontweight='bold',
                ha='center', va='center', zorder=8)

    trace = simulate_tour(registry, planner)
    ax.plot([p[0] for p in trace], [p[1] for p in trace], color=PATH, lw=2.1,
            zorder=9, solid_capstyle='round')

    home = registry['world']['home_pose']
    ax.plot(home[0], home[1], marker='s', ms=11, color='#e0a800',
            markeredgecolor=INK, markeredgewidth=1.0, zorder=10)
    ax.text(home[0] + 0.28, home[1] - 0.38, 'STAGING\nDOCK', fontsize=8.5,
            color=INK, fontweight='bold', va='top')

    ax.set_xlim(bounds['x_min'] - 0.6, bounds['x_max'] + 0.6)
    ax.set_ylim(bounds['y_min'] - 0.6, bounds['y_max'] + 0.9)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)', color=MUTED)
    ax.set_ylabel('y (m)', color=MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)
    for spine in ax.spines.values():
        spine.set_color('#dfe3e7')
    ax.grid(True, color='#e7eaed', lw=0.6)
    ax.set_axisbelow(True)

    ax.set_title(' ', pad=2)
    fig.text(0.055, 0.973, 'secure_station_dispatch · facility layout and planned tour',
             color=INK, fontsize=14.5, fontweight='bold', ha='left', va='top')
    fig.text(0.055, 0.948,
             f'berth standoff {board["standoff_m"]:.2f} m  ·  '
             f'{board["marker_size_m"]:.2f} m marker on a {board["board_size_m"]:.2f} m board  ·  '
             f'obstacle inflation {planner.inflation:.2f} m (shaded)  ·  '
             f'red = the path the control law actually drives',
             color=MUTED, fontsize=9.5, ha='left', va='top')

    fig.tight_layout(rect=(0, 0, 1, 0.935))
    fig.savefig(out_path, dpi=170, facecolor='white')
    plt.close(fig)
    return out_path


def render_legacy(registry, out_path):
    """The original layout, and the leg that drove through the Assembly panel."""
    berths = {i: tuple(s['expected_pose'][:2]) for i, s in sorted(registry['stations'].items())}
    panels = [Rect(f'panel_{i}', x, y + 0.40, 0.60, 0.01) for i, (x, y) in berths.items()]
    planner = PathPlanner(panels, inflation=0.43, hard_inflation=0.28)

    fig, ax = plt.subplots(figsize=(8.4, 8.8))
    fig.patch.set_facecolor('white')

    for rect in panels:
        draw_rect(ax, rect.inflated(0.28), facecolor='#f3c9c6', edgecolor='none', z=1)
        draw_rect(ax, Rect(rect.name, rect.cx, rect.cy, rect.dx, 0.06),
                  facecolor=STRUCTURE, edgecolor='none', z=3)

    order = sorted(berths)
    cursor = (0.0, 0.0)
    for marker_id in order:
        goal = berths[marker_id]
        colour = PATH if planner.blockers(cursor, goal,
                                          inflation=planner.hard_inflation) else '#5a6570'
        ax.annotate('', xy=goal, xytext=cursor,
                    arrowprops=dict(arrowstyle='-|>', color=colour, lw=2.4,
                                    shrinkA=0, shrinkB=0), zorder=6)
        cursor = goal

    for marker_id, (x, y) in berths.items():
        ax.plot(x, y, 'o', ms=9, color=ACCENTS[marker_id], zorder=7)
        ax.text(x + 0.16, y - 0.30,
                f'{registry["stations"][marker_id]["station_name"].replace(" Station", "")}\n'
                f'[{x:.1f}, {y:.1f}]',
                fontsize=8.5, color=INK, va='top')

    ax.plot(0, 0, 's', ms=10, color='#e0a800', markeredgecolor=INK, zorder=7)
    ax.annotate('the robot wedges here:\nAssembly → Storage drives straight\n'
                'through the Assembly panel at [4.5, 1.4]',
                xy=(4.5, 1.4), xytext=(1.7, 3.5), fontsize=9.5, color=PATH,
                fontweight='bold',
                arrowprops=dict(arrowstyle='-|>', color=PATH, lw=1.4), zorder=9)

    ax.set_xlim(-1.0, 6.0)
    ax.set_ylim(-1.0, 6.6)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)', color=MUTED)
    ax.set_ylabel('y (m)', color=MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)
    for spine in ax.spines.values():
        spine.set_color('#dfe3e7')
    ax.grid(True, color='#eef1f3', lw=0.6)
    ax.set_axisbelow(True)
    ax.set_title(' ', pad=2)
    fig.text(0.07, 0.972, 'Original Phase 3 layout — the collision that stopped the tour',
             color=INK, fontsize=13.5, fontweight='bold', ha='left', va='top')
    fig.text(0.07, 0.946, 'panels 0.40 m in +Y of every berth, solid collision boxes, '
                          'straight-line legs, no stall detection',
             color=MUTED, fontsize=9, ha='left', va='top')

    fig.tight_layout(rect=(0, 0, 1, 0.930))
    fig.savefig(out_path, dpi=170, facecolor='white')
    plt.close(fig)
    return out_path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', default=os.path.join(REPO, 'config', 'station_registry.yaml'))
    parser.add_argument('--out-dir', default=os.path.join(REPO, 'docs'))
    args = parser.parse_args(argv)

    with open(args.registry) as handle:
        registry = yaml.safe_load(handle)

    os.makedirs(args.out_dir, exist_ok=True)
    print(f'  wrote {render_current(registry, os.path.join(args.out_dir, "facility_map.png"))}')
    print(f'  wrote {render_legacy(registry, os.path.join(args.out_dir, "legacy_failure.png"))}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
