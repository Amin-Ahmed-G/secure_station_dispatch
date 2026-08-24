"""
gazebo_demo.launch.py
─────────────────────
Phase 3 Demo Polish Launch File.

Brings up:
  1. Gazebo Sim with industrial_station_world.sdf
  2. ros_gz_bridge (bridges camera images, cmd_vel, odom)
  3. task_simulator_node (action server)
  4. dispatcher_node (state machine with enable_security:=true)
  5. marker_detector_node (ArUco detector subscribing to camera feed)
  6. waypoint_tour_node (scripted 4-station tour)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('secure_station_dispatch')
    world_path = os.path.join(pkg_share, 'worlds', 'industrial_station_world.sdf')
    registry_path = os.path.join(pkg_share, 'config', 'station_registry.yaml')
    log_path = 'logs/dispatch_events.jsonl'

    # 1. Gazebo Sim headless / GUI process
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 2. ROS GZ Bridge for Camera, cmd_vel, odom
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    # 3. Core dispatch nodes
    task_simulator = Node(
        package='secure_station_dispatch',
        executable='task_simulator',
        name='task_simulator_node',
        output='screen',
    )

    dispatcher = Node(
        package='secure_station_dispatch',
        executable='dispatcher',
        name='dispatcher_node',
        output='screen',
        parameters=[{
            'registry_path': registry_path,
            'log_path': log_path,
            'enable_security': True,
        }],
    )

    marker_detector = Node(
        package='secure_station_dispatch',
        executable='marker_detector',
        name='marker_detector_node',
        output='screen',
        parameters=[{
            'registry_path': registry_path,
            'image_topic': '/camera/image_raw',
        }],
    )

    # 4. Waypoint Tour Node
    waypoint_tour = Node(
        package='secure_station_dispatch',
        executable='waypoint_tour',
        name='waypoint_tour_node',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg='=== secure_station_dispatch Phase 3 Gazebo Demo launching ==='),
        gazebo,
        bridge,
        task_simulator,
        TimerAction(period=2.0, actions=[dispatcher]),
        TimerAction(period=3.0, actions=[marker_detector]),
        TimerAction(period=5.0, actions=[waypoint_tour]),
    ])
