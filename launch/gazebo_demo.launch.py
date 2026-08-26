"""
gazebo_demo.launch.py
─────────────────────
Phase 3 end-to-end demo: Gazebo Sim + perception + dispatch + scripted tour.

Brings up, in order:
  1. Gazebo Sim with ``worlds/industrial_station_world.sdf`` (GUI or headless)
  2. ``ros_gz_bridge`` for /clock, /camera/image_raw, /cmd_vel and /odom
  3. ``task_simulator``  — StationTask action server
  4. ``dispatcher``      — state machine + action client, security enabled
  5. ``marker_detector`` — ArUco detection on the mast camera feed
  6. ``waypoint_tour``   — obstacle-aware tour of all four bays

Arguments
─────────
  gui:=true|false              run the Gazebo GUI (default true)
  use_sim_time:=true|false     drive node clocks from /clock (default false)
  enable_security:=true|false  Phase 2 HMAC + pose checks (default true)
  run_tour:=true|false         start the scripted tour (default true)
  shutdown_on_finish:=true|false  tear the stack down when the tour ends (default true)
  registry_path:=PATH          override the station registry
  log_path:=PATH               dispatch event log (default logs/dispatch_events.jsonl)

Examples
────────
  ros2 launch secure_station_dispatch gazebo_demo.launch.py
  ros2 launch secure_station_dispatch gazebo_demo.launch.py gui:=false
  ros2 launch secure_station_dispatch gazebo_demo.launch.py run_tour:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo,
    RegisterEventHandler, SetEnvironmentVariable, TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('secure_station_dispatch')
    default_world = os.path.join(pkg_share, 'worlds', 'industrial_station_world.sdf')
    default_registry = os.path.join(pkg_share, 'config', 'station_registry.yaml')

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_security = LaunchConfiguration('enable_security')
    run_tour = LaunchConfiguration('run_tour')
    shutdown_on_finish = LaunchConfiguration('shutdown_on_finish')
    registry_path = LaunchConfiguration('registry_path')
    log_path = LaunchConfiguration('log_path')
    report_path = LaunchConfiguration('report_path')

    declare = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Gazebo GUI; false runs server-only.'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='World SDF to load.'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Drive node clocks from the bridged /clock.'),
        DeclareLaunchArgument('enable_security', default_value='true',
                              description='Phase 2 HMAC + pose-consistency checks.'),
        DeclareLaunchArgument('run_tour', default_value='true',
                              description='Start the scripted waypoint tour.'),
        DeclareLaunchArgument('shutdown_on_finish', default_value='true',
                              description='Shut the stack down when the tour exits.'),
        DeclareLaunchArgument('registry_path', default_value=default_registry,
                              description='station_registry.yaml to use.'),
        DeclareLaunchArgument('log_path', default_value='logs/dispatch_events.jsonl',
                              description='Dispatcher JSONL event log.'),
        DeclareLaunchArgument('report_path', default_value='logs/tour_report.json',
                              description='Per-station tour report written by the tour node.'),
    ]

    # Marker textures are referenced by absolute path from the generated world,
    # but exporting the share directory as a Gazebo resource root keeps relative
    # and model:// lookups working too, and costs nothing when unused.
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[PathJoinSubstitution([pkg_share, 'models']), ':',
               PathJoinSubstitution([pkg_share, 'worlds']), ':',
               os.environ.get('GZ_SIM_RESOURCE_PATH', '')],
    )

    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '2', world],
        output='screen',
        condition=IfCondition(gui),
    )
    gazebo_headless = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', '-v', '2', world],
        output='screen',
        condition=UnlessCondition(gui),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen',
    )

    task_simulator = Node(
        package='secure_station_dispatch',
        executable='task_simulator',
        name='task_simulator_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    dispatcher = Node(
        package='secure_station_dispatch',
        executable='dispatcher',
        name='dispatcher_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'registry_path': registry_path,
            'log_path': log_path,
            'enable_security': enable_security,
        }],
    )

    marker_detector = Node(
        package='secure_station_dispatch',
        executable='marker_detector',
        name='marker_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'registry_path': registry_path,
            'image_topic': '/camera/image_raw',
        }],
    )

    waypoint_tour = Node(
        package='secure_station_dispatch',
        executable='waypoint_tour',
        name='waypoint_tour_node',
        output='screen',
        condition=IfCondition(run_tour),
        parameters=[{
            'use_sim_time': use_sim_time,
            'registry_path': registry_path,
            'report_path': report_path,
        }],
    )

    # The tour node exits non-zero if any bay was missed, so its exit is the
    # natural end of the demo — and a clean teardown means the report is the
    # last thing on screen instead of scrolling away under Gazebo chatter.
    stop_when_done = RegisterEventHandler(
        OnProcessExit(
            target_action=waypoint_tour,
            on_exit=[LogInfo(msg='Waypoint tour finished — shutting the demo down.'),
                     EmitEvent(event=Shutdown(reason='waypoint tour complete'))],
        ),
        condition=IfCondition(shutdown_on_finish),
    )

    return LaunchDescription([
        *declare,
        resource_path,
        LogInfo(msg='═══ secure_station_dispatch · Phase 3 Gazebo demo ═══'),
        gazebo_gui,
        gazebo_headless,
        bridge,
        # Gazebo needs a moment to spawn before the bridge sees topics, and the
        # action server must be up before the dispatcher starts looking for it.
        TimerAction(period=3.0, actions=[task_simulator]),
        TimerAction(period=5.0, actions=[dispatcher]),
        TimerAction(period=6.0, actions=[marker_detector]),
        TimerAction(period=9.0, actions=[waypoint_tour]),
        stop_when_done,
    ])
