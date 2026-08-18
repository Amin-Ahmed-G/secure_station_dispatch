"""
station_dispatch.launch.py
──────────────────────────
Launches all three Phase 1 nodes together:
  1. task_simulator_node  (action server — must start first)
  2. dispatcher_node      (action client + marker subscriber)
  3. marker_detector_node (camera subscriber + marker publisher)

Also includes commented-out nodes for the mock image publisher and test scripts.

Usage:
  ros2 launch secure_station_dispatch station_dispatch.launch.py
  ros2 launch secure_station_dispatch station_dispatch.launch.py \
      enable_security:=true   # Phase 2 only
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('secure_station_dispatch')
    registry_path = os.path.join(pkg_share, 'config', 'station_registry.yaml')
    log_path = 'logs/dispatch_events.jsonl'   # relative to process CWD

    # ── Launch arguments ───────────────────────────────────────────────────────
    enable_security_arg = DeclareLaunchArgument(
        'enable_security',
        default_value='false',
        description='Enable Phase 2 HMAC + pose security checks'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Image topic for ArUco detection'
    )

    enable_security = LaunchConfiguration('enable_security')
    image_topic     = LaunchConfiguration('image_topic')

    # ── Nodes ──────────────────────────────────────────────────────────────────

    # 1. Task Simulator (action server) — start first so the dispatcher can connect
    task_simulator = Node(
        package='secure_station_dispatch',
        executable='task_simulator',
        name='task_simulator_node',
        output='screen',
        emulate_tty=True,
    )

    # 2. Dispatcher (state machine + action client) — start after action server
    dispatcher = Node(
        package='secure_station_dispatch',
        executable='dispatcher',
        name='dispatcher_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'registry_path':     registry_path,
            'log_path':          log_path,
            'enable_security':   enable_security,
        }],
    )

    # 3. Marker Detector — start last (needs dispatcher already subscribed)
    marker_detector = Node(
        package='secure_station_dispatch',
        executable='marker_detector',
        name='marker_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'registry_path': registry_path,
            'image_topic':   image_topic,
        }],
    )

    # Give the action server 1 second head-start before bringing up the rest
    delayed_dispatcher = TimerAction(period=1.0, actions=[dispatcher])
    delayed_detector   = TimerAction(period=2.0, actions=[marker_detector])

    return LaunchDescription([
        enable_security_arg,
        image_topic_arg,
        LogInfo(msg='=== secure_station_dispatch Phase 1 launching ==='),
        task_simulator,
        delayed_dispatcher,
        delayed_detector,
    ])
