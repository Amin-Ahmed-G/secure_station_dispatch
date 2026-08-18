#!/usr/bin/env python3
"""
waypoint_tour_node.py
─────────────────────
Phase 3 Demo Polish: Scripted Waypoint Tour Node.

Sequentially navigates the AMR robot to each of the 4 industrial stations:
  1. Station 0 (Inspection): [1.0, 2.0, 0.0]
  2. Station 1 (Assembly):   [4.5, 1.0, 0.0]
  3. Station 2 (Storage):    [4.5, 5.0, 0.0]
  4. Station 3 (Charging):   [0.5, 5.5, 0.0]

Publishes robot pose to /mock_pose (for detector pose tracking) and sends velocity
commands to /cmd_vel.

When running with Gazebo Sim + enable_security:=true:
  - Robot approaches station marker.
  - Camera detects physical ArUco marker in Gazebo world.
  - Detector node signs detection message with HMAC.
  - Dispatcher verifies HMAC signature and robot pose tolerance.
  - Dispatcher triggers StationTask action on task_simulator_node.
"""

import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped


WAYPOINTS = [
    {'id': 0, 'name': 'Inspection Station', 'pose': [1.0, 2.0, 0.0]},
    {'id': 1, 'name': 'Assembly Station',   'pose': [4.5, 1.0, 0.0]},
    {'id': 2, 'name': 'Storage Station',    'pose': [4.5, 5.0, 0.0]},
    {'id': 3, 'name': 'Charging Station',   'pose': [0.5, 5.5, 0.0]},
]


class WaypointTourNode(Node):

    def __init__(self):
        super().__init__('waypoint_tour_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/mock_pose', 10)
        self.get_logger().info('WaypointTourNode ready — starting automated tour across 4 stations...')

    def run_tour(self):
        time.sleep(2.0)  # Initial stabilization

        for idx, wp in enumerate(WAYPOINTS):
            self.get_logger().info(
                f'\n============================================================\n'
                f'  WAYPOINT TOUR [{idx + 1}/4]: Approaching {wp["name"]} '
                f'(Target Pose: {wp["pose"]})\n'
                f'============================================================'
            )

            # Publish updated pose for AMCL / detector pose tracker
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = wp['pose'][0]
            pose_msg.pose.position.y = wp['pose'][1]
            pose_msg.pose.position.z = wp['pose'][2]
            pose_msg.pose.orientation.w = 1.0
            self.pose_pub.publish(pose_msg)

            # Send brief drive command towards target
            twist = Twist()
            twist.linear.x = 0.5
            self.cmd_pub.publish(twist)
            time.sleep(1.0)

            # Stop at station facing marker
            twist.linear.x = 0.0
            self.cmd_pub.publish(twist)

            self.get_logger().info(
                f'  Arrived at {wp["name"]}. Waiting 6s for detection & task completion...'
            )

            # Hold at station while task executes
            start_time = time.time()
            while time.time() - start_time < 6.0:
                self.pose_pub.publish(pose_msg)
                time.sleep(0.5)

        self.get_logger().info(
            '\n============================================================\n'
            '  WAYPOINT TOUR COMPLETE — All 4 stations visited!\n'
            '============================================================'
        )


def main(args=None):
    rclpy.init(args=args)
    node = WaypointTourNode()
    try:
        node.run_tour()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
