#!/usr/bin/env python3
"""
check_world_sync.py
───────────────────
Verifies that ``worlds/industrial_station_world.sdf`` and
``config/station_registry.yaml`` still describe the same facility.

This is the guard rail for the original Phase 3 defect.  The robot drove into
the Assembly marker panel because the world contained a solid obstacle the
planner had never been told about; nothing in the build would have caught that.
Now every collision volume in the world must lie inside a footprint the registry
declares, and every declared footprint must be backed by real geometry.

Run standalone::

    python3 scripts/check_world_sync.py            # exits non-zero on drift

It is also executed by ``test/test_waypoint_planner.py`` so a plain test run
catches the drift too.
"""

from __future__ import annotations

import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_WORLD = os.path.join(REPO, 'worlds', 'industrial_station_world.sdf')
DEFAULT_REGISTRY = os.path.join(REPO, 'config', 'station_registry.yaml')

# Decorative models that carry no collision geometry by design.
DECOR_PREFIXES = ('ground_plane', 'facility_floor', 'staging_dock', 'berth_decal_')
ROBOT_MODEL = 'amr_robot'

TOLERANCE = 1e-4


def _parse_pose(text: str | None) -> tuple[float, float, float, float]:
    """Return (x, y, z, yaw) from an SDF pose string."""
    if not text:
        return 0.0, 0.0, 0.0, 0.0
    values = [float(v) for v in text.split()]
    values += [0.0] * (6 - len(values))
    return values[0], values[1], values[2], values[5]


def _box_size(collision: ET.Element) -> tuple[float, float, float] | None:
    box = collision.find('./geometry/box/size')
    if box is None or not box.text:
        return None
    sx, sy, sz = (float(v) for v in box.text.split())
    return sx, sy, sz


def world_aabbs(world_path: str) -> dict[str, tuple[float, float, float, float]]:
    """World-frame XY bounding box of every static model's collision geometry."""
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    if world is None:
        raise SystemExit(f'{world_path}: no <world> element')

    boxes: dict[str, tuple[float, float, float, float]] = {}
    for model in world.findall('model'):
        name = model.get('name', '')
        if name == ROBOT_MODEL or name.startswith(DECOR_PREFIXES):
            continue

        mx, my, _mz, myaw = _parse_pose(model.findtext('pose'))
        cos_y, sin_y = math.cos(myaw), math.sin(myaw)

        xs: list[float] = []
        ys: list[float] = []
        for link in model.findall('link'):
            lx, ly, _lz, _ = _parse_pose(link.findtext('pose'))
            for collision in link.findall('collision'):
                size = _box_size(collision)
                if size is None:
                    continue
                cx, cy, _cz, _ = _parse_pose(collision.findtext('pose'))
                hx, hy = size[0] / 2.0, size[1] / 2.0
                for dx, dy in ((-hx, -hy), (-hx, hy), (hx, -hy), (hx, hy)):
                    px, py = lx + cx + dx, ly + cy + dy
                    xs.append(mx + px * cos_y - py * sin_y)
                    ys.append(my + px * sin_y + py * cos_y)
        if xs:
            boxes[name] = (min(xs), min(ys), max(xs), max(ys))
    return boxes


def registry_footprints(registry: dict) -> dict[str, tuple[float, float, float, float]]:
    """Declared XY footprints, keyed by the world model name they belong to."""
    out: dict[str, tuple[float, float, float, float]] = {}
    world = registry.get('world', {}) or {}

    for entry in world.get('obstacles', []) or []:
        cx, cy = entry['center']
        dx, dy = entry['size']
        out[entry['name']] = (cx - dx / 2, cy - dy / 2, cx + dx / 2, cy + dy / 2)

    for marker_id, station in sorted((registry.get('stations', {}) or {}).items()):
        pose = station['marker_pose']
        dx, dy = station['bay_footprint']
        key = f'bay_{marker_id}_{station["station_name"].split()[0].lower()}'
        out[key] = (pose[0] - dx / 2, pose[1] - dy / 2,
                    pose[0] + dx / 2, pose[1] + dy / 2)
    return out


def check(world_path: str = DEFAULT_WORLD,
          registry_path: str = DEFAULT_REGISTRY) -> list[str]:
    """Return a list of human-readable problems; empty means the two agree."""
    with open(registry_path) as handle:
        registry = yaml.safe_load(handle)

    actual = world_aabbs(world_path)
    declared = registry_footprints(registry)
    problems: list[str] = []

    for name, box in sorted(actual.items()):
        if name not in declared:
            problems.append(
                f'{name}: has collision geometry in the world but no footprint in '
                f'the registry — the planner would drive straight through it')
            continue
        d = declared[name]
        if (box[0] < d[0] - TOLERANCE or box[1] < d[1] - TOLERANCE
                or box[2] > d[2] + TOLERANCE or box[3] > d[3] + TOLERANCE):
            problems.append(
                f'{name}: collision AABB '
                f'[{box[0]:.3f}, {box[1]:.3f}]–[{box[2]:.3f}, {box[3]:.3f}] '
                f'escapes the declared footprint '
                f'[{d[0]:.3f}, {d[1]:.3f}]–[{d[2]:.3f}, {d[3]:.3f}]')

    for name in sorted(declared):
        if name not in actual:
            problems.append(
                f'{name}: declared as an obstacle in the registry but has no '
                f'collision geometry in the world')

    return problems


def main(argv=None) -> int:
    import argparse
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', default=DEFAULT_WORLD)
    parser.add_argument('--registry', default=DEFAULT_REGISTRY)
    args = parser.parse_args(argv)

    problems = check(args.world, args.registry)
    if problems:
        print('WORLD / REGISTRY DRIFT DETECTED', file=sys.stderr)
        for problem in problems:
            print(f'  ✗ {problem}', file=sys.stderr)
        return 1

    boxes = world_aabbs(args.world)
    print(f'  ✓ {len(boxes)} collision models all inside their declared footprints')
    for name, box in sorted(boxes.items()):
        print(f'      {name:<22} [{box[0]:6.2f}, {box[1]:6.2f}] – '
              f'[{box[2]:6.2f}, {box[3]:6.2f}]')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
