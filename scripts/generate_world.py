#!/usr/bin/env python3
"""
generate_world.py
─────────────────
Emits ``worlds/industrial_station_world.sdf`` from ``config/station_registry.yaml``.

Why the world is generated rather than hand-written
───────────────────────────────────────────────────
The Phase 3 collision bug was a synchronisation failure: the world placed a solid
marker panel 0.40 m in front of every berth, and nothing that planned the robot's
motion knew those panels existed.  Hand-maintaining two descriptions of the same
facility is what allowed them to disagree.

So the registry is now the only place station geometry is written down, and this
script projects it into SDF.  Bay structures are laid out from
``stations[*].marker_pose`` / ``marker_normal``, walls, racks and pallets from
``world.obstacles``, and every collision volume is guaranteed to sit inside the
footprint the navigation planner inflates.  ``scripts/check_world_sync.py``
re-verifies that invariant without regenerating anything.

Usage:
    python3 scripts/generate_world.py                     # regenerate in place
    python3 scripts/generate_world.py --texture-root PATH # relocate textures
    python3 scripts/generate_world.py --stdout            # preview
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import yaml

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ── Bay template, expressed in the bay's local frame ──────────────────────────
# Local -Y is the readable face.  Every bay is the same model, yawed so that its
# readable face points along the station's marker_normal, which keeps the ArUco
# texture mapping identical for all four boards.
BOARD_THICKNESS = 0.04
POST_SECTION = 0.10
POST_HEIGHT = 1.75
POST_SPAN = 0.60          # ± local X offset of the frame uprights
PLINTH = (1.40, 0.19, 0.12)
PLINTH_Y = 0.055
NAMEPLATE = (1.10, 0.03, 0.18)
INFOPLATE = (1.10, 0.03, 0.14)
PLATE_Y = -0.03
PLATE_GAP = 0.05

RACKS = {
    'rack_centre': {'template': (1.60, 0.90), 'yaw': 0.0, 'height': 1.80, 'levels': 3},
    'rack_east':   {'template': (1.80, 0.90), 'yaw': math.pi / 2, 'height': 1.80, 'levels': 3},
}
PALLETS = ('pallet_west', 'pallet_ne')

ACCENTS = {
    0: (0.12, 0.44, 0.70),
    1: (0.71, 0.42, 0.08),
    2: (0.18, 0.49, 0.31),
    3: (0.42, 0.25, 0.63),
}


def rgba(c, a=1.0):
    return f'{c[0]:.3f} {c[1]:.3f} {c[2]:.3f} {a:.2f}'


def solid(name, size, pose, colour, *, collision=True, specular=(0.1, 0.1, 0.1)):
    """A coloured box visual, optionally with a matching collision."""
    sx, sy, sz = size
    geom = f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
    out = [f'        <visual name="{name}_vis">',
           f'          <pose>{pose}</pose>',
           f'          {geom}',
           '          <material>',
           f'            <ambient>{rgba([v * 0.55 for v in colour])}</ambient>',
           f'            <diffuse>{rgba(colour)}</diffuse>',
           f'            <specular>{rgba(specular)}</specular>',
           '          </material>',
           '        </visual>']
    if collision:
        out += [f'        <collision name="{name}_col">',
                f'          <pose>{pose}</pose>',
                f'          {geom}',
                '        </collision>']
    return out


def textured(name, size, pose, texture, *, collision=False):
    """A box visual carrying a PBR albedo map (the signage plates)."""
    sx, sy, sz = size
    geom = f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'
    out = [f'        <visual name="{name}_vis">',
           f'          <pose>{pose}</pose>',
           f'          {geom}',
           '          <material>',
           '            <ambient>0.85 0.85 0.85 1</ambient>',
           '            <diffuse>1 1 1 1</diffuse>',
           '            <specular>0.02 0.02 0.02 1</specular>',
           '            <pbr><metal>',
           f'              <albedo_map>{texture}</albedo_map>',
           '              <metalness>0.0</metalness>',
           '              <roughness>0.9</roughness>',
           '            </metal></pbr>',
           '          </material>',
           '        </visual>']
    if collision:
        out += [f'        <collision name="{name}_col">',
                f'          <pose>{pose}</pose>',
                f'          {geom}',
                '        </collision>']
    return out


def static_model(name, pose, body):
    return ['\n    <model name="%s">' % name,
            '      <static>true</static>',
            f'      <pose>{pose}</pose>',
            '      <link name="link">',
            *body,
            '      </link>',
            '    </model>']


# ═══════════════════════════════════════════════════════════════════════════════
def bay_model(marker_id: int, station: dict, texture_root: str) -> list[str]:
    """One station bay: plinth, frame uprights, marker board and signage plates."""
    mx, my, mz = station['marker_pose']
    nx, ny = station['marker_normal']

    # Yaw that maps the template's readable face (local -Y) onto marker_normal.
    yaw = math.atan2(nx, -ny)

    board = station.get('_board_size', 1.10)
    accent = ACCENTS.get(marker_id, (0.4, 0.4, 0.4))
    steel = (0.36, 0.39, 0.43)
    body: list[str] = []

    body += solid(f'bay{marker_id}_plinth', PLINTH,
                  f'0 {PLINTH_Y:.3f} {PLINTH[2] / 2:.3f} 0 0 0', steel)
    for sign, tag in ((-1, 'l'), (1, 'r')):
        body += solid(f'bay{marker_id}_post_{tag}',
                      (POST_SECTION, POST_SECTION, POST_HEIGHT),
                      f'{sign * POST_SPAN:.3f} 0.05 {POST_HEIGHT / 2:.3f} 0 0 0', steel)
    body += solid(f'bay{marker_id}_cap', (2 * POST_SPAN + POST_SECTION, 0.10, 0.06),
                  f'0 0.05 {POST_HEIGHT + 0.03:.3f} 0 0 0', accent)

    # Marker board — the only textured surface the detector must resolve.
    body += textured(f'bay{marker_id}_board', (board, BOARD_THICKNESS, board),
                     f'0 0 {mz:.3f} 0 0 0',
                     os.path.join(texture_root, f'marker_{marker_id}.png'),
                     collision=True)

    top = mz + board / 2 + PLATE_GAP + NAMEPLATE[2] / 2
    bottom = mz - board / 2 - PLATE_GAP - INFOPLATE[2] / 2
    body += textured(f'bay{marker_id}_nameplate', NAMEPLATE,
                     f'0 {PLATE_Y:.3f} {top:.3f} 0 0 0',
                     os.path.join(texture_root, f'nameplate_{marker_id}.png'))
    body += textured(f'bay{marker_id}_infoplate', INFOPLATE,
                     f'0 {PLATE_Y:.3f} {bottom:.3f} 0 0 0',
                     os.path.join(texture_root, f'infoplate_{marker_id}.png'))

    name = f'bay_{marker_id}_{station["station_name"].split()[0].lower()}'
    return static_model(name, f'{mx:.3f} {my:.3f} 0 0 0 {yaw:.6f}', body)


def wall_model(entry: dict) -> list[str]:
    cx, cy = entry['center']
    dx, dy = entry['size']
    height = 2.20
    body = solid('wall', (dx, dy, height), f'0 0 {height / 2:.3f} 0 0 0', (0.80, 0.80, 0.78))
    # Safety kick-band along the base.
    body += solid('band', (dx * 1.002, dy * 1.002, 0.18), '0 0 0.09 0 0 0',
                  (0.30, 0.33, 0.36), collision=False)
    return static_model(entry['name'], f'{cx:.3f} {cy:.3f} 0 0 0 0', body)


def rack_model(entry: dict, spec: dict) -> list[str]:
    cx, cy = entry['center']
    length, depth = spec['template']
    height, levels = spec['height'], spec['levels']
    steel = (0.72, 0.35, 0.10)
    shelf = (0.55, 0.57, 0.60)

    body: list[str] = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            body += solid(f'upright_{sx}_{sy}', (0.09, 0.09, height),
                          f'{sx * (length / 2 - 0.06):.3f} {sy * (depth / 2 - 0.06):.3f} '
                          f'{height / 2:.3f} 0 0 0', steel)
    for level in range(levels):
        z = 0.12 + level * (height - 0.24) / max(levels - 1, 1)
        body += solid(f'shelf_{level}', (length - 0.04, depth - 0.04, 0.06),
                      f'0 0 {z:.3f} 0 0 0', shelf, collision=(level == 0))
        if level < levels - 1:
            body += solid(f'crate_{level}', (length * 0.3, depth * 0.6, 0.34),
                          f'{(level - 1) * length * 0.3:.3f} 0 {z + 0.20:.3f} 0 0 0',
                          (0.62, 0.50, 0.33), collision=False)
    return static_model(entry['name'], f'{cx:.3f} {cy:.3f} 0 0 0 {spec["yaw"]:.6f}', body)


def pallet_model(entry: dict) -> list[str]:
    cx, cy = entry['center']
    dx, dy = entry['size']
    body = solid('pallet', (dx, dy, 0.14), '0 0 0.07 0 0 0', (0.55, 0.42, 0.26))
    body += solid('load', (dx * 0.85, dy * 0.85, 0.46), '0 0 0.37 0 0 0', (0.68, 0.60, 0.45))
    body += solid('strap', (dx * 0.87, dy * 0.10, 0.48), '0 0 0.37 0 0 0',
                  (0.15, 0.16, 0.18), collision=False)
    return static_model(entry['name'], f'{cx:.3f} {cy:.3f} 0 0 0 0', body)


def berth_decal(marker_id: int, station: dict) -> list[str]:
    x, y = station['expected_pose'][0], station['expected_pose'][1]
    accent = ACCENTS.get(marker_id, (0.5, 0.5, 0.5))
    body = solid(f'pad{marker_id}', (1.10, 1.10, 0.006), '0 0 0.004 0 0 0',
                 accent, collision=False)
    body += solid(f'pad{marker_id}_inner', (0.86, 0.86, 0.008), '0 0 0.005 0 0 0',
                  (0.66, 0.66, 0.64), collision=False)
    return static_model(f'berth_decal_{marker_id}', f'{x:.3f} {y:.3f} 0 0 0 0', body)


def robot_model(home: list[float]) -> list[str]:
    """Differential-drive AMR with a 0.85 m sensor mast.

    The mast is the other half of the detection fix: the original camera sat at
    0.25 m, so the marker board could never be framed squarely.  Optical centre
    is 0.15 m ahead of the robot datum and level with the board centre, which is
    exactly the geometry ``navigation.perception_budget`` is written against.
    """
    yaw = home[2] if len(home) > 2 else 0.0
    return f"""
    <!-- ═══════════════════════════════════════════════════════════════════
         AMR — 0.40 x 0.30 m differential-drive chassis, 0.85 m sensor mast
    ═══════════════════════════════════════════════════════════════════ -->
    <model name="amr_robot">
      <pose>{home[0]:.3f} {home[1]:.3f} 0.080 0 0 {yaw:.6f}</pose>

      <link name="chassis">
        <pose>0 0 0.05 0 0 0</pose>
        <inertial>
          <mass>8.0</mass>
          <inertia>
            <ixx>0.10</ixx><ixy>0</ixy><ixz>0</ixz>
            <iyy>0.14</iyy><iyz>0</iyz><izz>0.18</izz>
          </inertia>
        </inertial>

        <collision name="chassis_col">
          <geometry><box><size>0.40 0.30 0.16</size></box></geometry>
        </collision>
        <visual name="chassis_vis">
          <geometry><box><size>0.40 0.30 0.16</size></box></geometry>
          <material>
            <ambient>0.06 0.16 0.34 1</ambient>
            <diffuse>0.11 0.30 0.62 1</diffuse>
            <specular>0.25 0.25 0.25 1</specular>
          </material>
        </visual>

        <!-- Hazard band + status beacon -->
        <visual name="hazard_band_vis">
          <pose>0 0 -0.06 0 0 0</pose>
          <geometry><box><size>0.406 0.306 0.045</size></box></geometry>
          <material>
            <ambient>0.55 0.45 0.05 1</ambient>
            <diffuse>0.95 0.78 0.10 1</diffuse>
          </material>
        </visual>
        <visual name="bumper_vis">
          <pose>0.205 0 -0.03 0 0 0</pose>
          <geometry><box><size>0.03 0.28 0.08</size></box></geometry>
          <material>
            <ambient>0.20 0.20 0.20 1</ambient>
            <diffuse>0.28 0.28 0.30 1</diffuse>
          </material>
        </visual>

        <!-- Sensor mast -->
        <collision name="mast_col">
          <pose>0.06 0 0.375 0 0 0</pose>
          <geometry><box><size>0.09 0.09 0.59</size></box></geometry>
        </collision>
        <visual name="mast_vis">
          <pose>0.06 0 0.375 0 0 0</pose>
          <geometry><box><size>0.09 0.09 0.59</size></box></geometry>
          <material>
            <ambient>0.18 0.19 0.21 1</ambient>
            <diffuse>0.30 0.32 0.35 1</diffuse>
          </material>
        </visual>
        <visual name="camera_head_vis">
          <pose>0.10 0 0.72 0 0 0</pose>
          <geometry><box><size>0.10 0.16 0.10</size></box></geometry>
          <material>
            <ambient>0.05 0.05 0.06 1</ambient>
            <diffuse>0.12 0.13 0.15 1</diffuse>
            <specular>0.4 0.4 0.4 1</specular>
          </material>
        </visual>
        <visual name="lens_vis">
          <pose>0.151 0 0.72 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.028</radius><length>0.012</length></cylinder></geometry>
          <material>
            <ambient>0.02 0.04 0.08 1</ambient>
            <diffuse>0.05 0.12 0.24 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>

        <!-- Forward mast camera: optical centre 0.15 m ahead of the robot
             datum and 0.85 m above the floor, i.e. level with the boards. -->
        <sensor name="front_camera" type="camera">
          <pose>0.15 0 0.72 0 0 0</pose>
          <topic>/camera/image_raw</topic>
          <update_rate>15</update_rate>
          <always_on>true</always_on>
          <visualize>true</visualize>
          <camera>
            <horizontal_fov>1.047</horizontal_fov>
            <image>
              <width>640</width><height>480</height>
              <format>R8G8B8</format>
            </image>
            <clip><near>0.05</near><far>25.0</far></clip>
            <noise><type>gaussian</type><mean>0.0</mean><stddev>0.004</stddev></noise>
          </camera>
        </sensor>
      </link>

      <!-- Drive wheels: separation 0.31 m, radius 0.08 m -->
      <link name="left_wheel">
        <pose>0 0.155 0 -1.5707 0 0</pose>
        <inertial>
          <mass>0.6</mass>
          <inertia><ixx>0.0012</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0012</iyy><iyz>0</iyz><izz>0.0019</izz></inertia>
        </inertial>
        <collision name="col">
          <geometry><cylinder><radius>0.08</radius><length>0.045</length></cylinder></geometry>
          <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface>
        </collision>
        <visual name="vis">
          <geometry><cylinder><radius>0.08</radius><length>0.045</length></cylinder></geometry>
          <material><ambient>0.08 0.08 0.09 1</ambient><diffuse>0.16 0.16 0.18 1</diffuse></material>
        </visual>
      </link>

      <link name="right_wheel">
        <pose>0 -0.155 0 -1.5707 0 0</pose>
        <inertial>
          <mass>0.6</mass>
          <inertia><ixx>0.0012</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0012</iyy><iyz>0</iyz><izz>0.0019</izz></inertia>
        </inertial>
        <collision name="col">
          <geometry><cylinder><radius>0.08</radius><length>0.045</length></cylinder></geometry>
          <surface><friction><ode><mu>1.2</mu><mu2>1.2</mu2></ode></friction></surface>
        </collision>
        <visual name="vis">
          <geometry><cylinder><radius>0.08</radius><length>0.045</length></cylinder></geometry>
          <material><ambient>0.08 0.08 0.09 1</ambient><diffuse>0.16 0.16 0.18 1</diffuse></material>
        </visual>
      </link>
__CASTERS__
      <joint name="left_wheel_joint" type="revolute">
        <parent>chassis</parent><child>left_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
          <dynamics><damping>0.01</damping></dynamics>
        </axis>
      </joint>
      <joint name="right_wheel_joint" type="revolute">
        <parent>chassis</parent><child>right_wheel</child>
        <axis>
          <xyz>0 0 1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
          <dynamics><damping>0.01</damping></dynamics>
        </axis>
      </joint>
__CASTER_JOINTS__
      <plugin filename="gz-sim-diff-drive-system"
              name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.31</wheel_separation>
        <wheel_radius>0.08</wheel_radius>
        <topic>/cmd_vel</topic>
        <odom_topic>/odom</odom_topic>
        <tf_topic>/tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>30</odom_publish_frequency>
        <max_linear_acceleration>0.8</max_linear_acceleration>
        <min_linear_acceleration>-0.8</min_linear_acceleration>
        <max_angular_acceleration>2.5</max_angular_acceleration>
        <min_angular_acceleration>-2.5</min_angular_acceleration>
      </plugin>

      <plugin filename="gz-sim-pose-publisher-system"
              name="gz::sim::systems::PosePublisher">
        <publish_link_pose>false</publish_link_pose>
        <publish_model_pose>true</publish_model_pose>
        <publish_nested_model_pose>true</publish_nested_model_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <update_frequency>20</update_frequency>
      </plugin>
    </model>
"""


def _casters() -> tuple[str, str]:
    """Four low-friction ball casters — two forward, two aft, so the chassis
    cannot pitch onto its bumper when the drive wheels brake."""
    links, joints = [], []
    for tag, x, y in (('fl', 0.15, 0.10), ('fr', 0.15, -0.10),
                      ('rl', -0.15, 0.10), ('rr', -0.15, -0.10)):
        links.append(f"""
      <link name="caster_{tag}">
        <pose>{x:.3f} {y:.3f} -0.040 0 0 0</pose>
        <inertial>
          <mass>0.15</mass>
          <inertia><ixx>0.0001</ixx><ixy>0</ixy><ixz>0</ixz><iyy>0.0001</iyy><iyz>0</iyz><izz>0.0001</izz></inertia>
        </inertial>
        <collision name="col">
          <geometry><sphere><radius>0.040</radius></sphere></geometry>
          <surface><friction><ode><mu>0.008</mu><mu2>0.008</mu2></ode></friction></surface>
        </collision>
        <visual name="vis">
          <geometry><sphere><radius>0.040</radius></sphere></geometry>
          <material><ambient>0.16 0.16 0.17 1</ambient><diffuse>0.26 0.26 0.28 1</diffuse></material>
        </visual>
      </link>""")
        joints.append(f"""
      <joint name="caster_{tag}_joint" type="ball">
        <parent>chassis</parent><child>caster_{tag}</child>
      </joint>""")
    return ''.join(links) + '\n', ''.join(joints) + '\n'


def build(registry: dict, texture_root: str) -> str:
    world = registry.get('world', {})
    bounds = world['bounds']
    home = world.get('home_pose', [0.0, 0.0, 0.0])
    board_size = world.get('marker_board', {}).get('board_size_m', 1.10)
    obstacles = {o['name']: o for o in world.get('obstacles', [])}

    floor_cx = (bounds['x_min'] + bounds['x_max']) / 2.0
    floor_cy = (bounds['y_min'] + bounds['y_max']) / 2.0
    floor_w = bounds['x_max'] - bounds['x_min']
    floor_h = bounds['y_max'] - bounds['y_min']

    parts: list[str] = []
    parts.append(f"""<?xml version="1.0" ?>
<!--
  industrial_station_world.sdf
  ───────────────────────────
  GENERATED FILE — do not edit by hand.
  Regenerate with:  python3 scripts/generate_world.py

  Every dimension below is projected from config/station_registry.yaml so the
  simulator, the navigation planner and the security pose checks cannot disagree
  about where a station is.  That disagreement is exactly what put the robot into
  the Assembly marker panel in the first Phase 3 build.

  Facility  : {floor_w:.1f} m x {floor_h:.1f} m, walled, four perimeter bays
  Berths    : robot datum 1.75 m from each board, square on to the marker
  Boards    : {board_size:.2f} m carrier, 0.70 m DICT_4X4_50 marker at 0.85 m
-->
<sdf version="1.8">
  <world name="industrial_station_world">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <gravity>0 0 -9.81</gravity>

    <scene>
      <ambient>0.55 0.56 0.58 1</ambient>
      <background>0.78 0.82 0.86 1</background>
      <grid>false</grid>
      <shadows>true</shadows>
    </scene>

    <!-- ═══════════════════════════════════════════════════════════════════
         Lighting — one key light plus two fills, so the marker boards are
         evenly lit from every approach heading.  Uneven lighting across a
         board is the classic cause of intermittent ArUco dropouts.
    ═══════════════════════════════════════════════════════════════════ -->
    <light type="directional" name="key_light">
      <cast_shadows>true</cast_shadows>
      <pose>{floor_cx:.2f} {floor_cy:.2f} 12 0 0 0</pose>
      <diffuse>0.85 0.85 0.84 1</diffuse>
      <specular>0.20 0.20 0.20 1</specular>
      <attenuation><range>60</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
      <direction>-0.35 -0.25 -0.90</direction>
    </light>
    <light type="directional" name="fill_east">
      <cast_shadows>false</cast_shadows>
      <pose>{bounds['x_max']:.2f} {floor_cy:.2f} 8 0 0 0</pose>
      <diffuse>0.32 0.33 0.35 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>60</range><constant>0.9</constant></attenuation>
      <direction>0.75 0.25 -0.60</direction>
    </light>
    <light type="directional" name="fill_north">
      <cast_shadows>false</cast_shadows>
      <pose>{floor_cx:.2f} {bounds['y_max']:.2f} 8 0 0 0</pose>
      <diffuse>0.30 0.31 0.33 1</diffuse>
      <specular>0.05 0.05 0.05 1</specular>
      <attenuation><range>60</range><constant>0.9</constant></attenuation>
      <direction>0.10 0.80 -0.60</direction>
    </light>

    <!-- ═══════════════════════════════════════════════════════════════════
         Ground and floor markings
    ═══════════════════════════════════════════════════════════════ -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>80 80</size></plane></geometry>
          <material>
            <ambient>0.38 0.39 0.40 1</ambient>
            <diffuse>0.52 0.53 0.54 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>
""")

    # Facility floor slab + hazard border
    floor_body = solid('slab', (floor_w, floor_h, 0.004), '0 0 0.002 0 0 0',
                       (0.66, 0.66, 0.64), collision=False)
    floor_body += solid('border', (floor_w - 0.5, floor_h - 0.5, 0.005),
                        '0 0 0.0035 0 0 0', (0.60, 0.60, 0.58), collision=False)
    parts.append('\n'.join(static_model('facility_floor',
                                        f'{floor_cx:.3f} {floor_cy:.3f} 0 0 0 0',
                                        floor_body)))

    # Staging pad at the dock
    dock_body = solid('pad', (1.30, 1.30, 0.006), '0 0 0.004 0 0 0', (0.90, 0.74, 0.10),
                      collision=False)
    dock_body += solid('pad_inner', (1.02, 1.02, 0.008), '0 0 0.005 0 0 0',
                       (0.62, 0.62, 0.60), collision=False)
    parts.append('\n'.join(static_model('staging_dock',
                                        f'{home[0]:.3f} {home[1]:.3f} 0 0 0 0', dock_body)))

    parts.append('\n    <!-- ── Perimeter walls ──────────────────────────────────── -->')
    for name in ('wall_west', 'wall_east', 'wall_south', 'wall_north'):
        if name in obstacles:
            parts.append('\n'.join(wall_model(obstacles[name])))

    parts.append('\n    <!-- ── Station bays (geometry from station_registry.yaml) ── -->')
    for marker_id, station in sorted(registry.get('stations', {}).items()):
        station = dict(station)
        station['_board_size'] = board_size
        parts.append('\n'.join(berth_decal(marker_id, station)))
        parts.append('\n'.join(bay_model(marker_id, station, texture_root)))

    parts.append('\n    <!-- ── Warehouse furniture (planner obstacles) ───────────── -->')
    for name, spec in RACKS.items():
        if name in obstacles:
            parts.append('\n'.join(rack_model(obstacles[name], spec)))
    for name in PALLETS:
        if name in obstacles:
            parts.append('\n'.join(pallet_model(obstacles[name])))

    caster_links, caster_joints = _casters()
    parts.append(robot_model(home)
                 .replace('__CASTERS__', caster_links)
                 .replace('__CASTER_JOINTS__', caster_joints))

    parts.append("""
    <!--
      Optional GUI preset.  Uncomment to open Gazebo looking down the aisle from
      the staging dock.  Left commented out because the available gz-gui plugin
      set varies between Gazebo releases, and a missing plugin yields a blank
      viewport rather than a clear error.

      <gui fullscreen="0">
        <plugin filename="MinimalScene" name="3D View">
          <gz-gui><property key="showTitleBar" type="bool">false</property></gz-gui>
          <camera_pose>-4.5 -4.5 6.5 0 0.62 0.85</camera_pose>
        </plugin>
        <plugin filename="GzSceneManager" name="Scene Manager"/>
        <plugin filename="InteractiveViewControl" name="Interactive view control"/>
        <plugin filename="WorldControl" name="World control"/>
        <plugin filename="WorldStats" name="World stats"/>
      </gui>
    -->

  </world>
</sdf>
""")
    return '\n'.join(parts)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Generate the Gazebo world from the registry.")
    parser.add_argument('--registry', default=os.path.join(REPO, 'config', 'station_registry.yaml'))
    parser.add_argument('--output', default=os.path.join(REPO, 'worlds', 'industrial_station_world.sdf'))
    parser.add_argument('--texture-root', default=os.path.join(REPO, 'models', 'materials', 'textures'),
                        help='absolute directory the <albedo_map> paths point at')
    parser.add_argument('--stdout', action='store_true')
    args = parser.parse_args(argv)

    with open(args.registry) as handle:
        registry = yaml.safe_load(handle)

    sdf = build(registry, os.path.abspath(args.texture_root))
    if args.stdout:
        sys.stdout.write(sdf)
        return 0

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, 'w') as handle:
        handle.write(sdf)
    print(f'  wrote {args.output}  ({len(sdf.splitlines())} lines)')
    print(f'  textures referenced from {os.path.abspath(args.texture_root)}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
