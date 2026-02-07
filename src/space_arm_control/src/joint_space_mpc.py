#!/usr/bin/env python3
"""
Simple joint-space MPC demo node.

Sends the arm through a sequence of predefined joint configurations
to demonstrate the MPC controller in microgravity.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Predefined waypoints (joint angles in radians)
WAYPOINTS = [
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],          # Home
    [0.5, -0.3, 0.6, 0.0, 0.2, 0.0],          # Reach right
    [-0.5, 0.3, -0.6, 0.0, -0.2, 0.0],        # Reach left
    [0.0, -0.8, 1.2, 0.0, 0.5, 0.0],          # Extend forward
    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],           # Home
]

JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']


class JointSpaceMPCDemo(Node):
    """Cycles through joint-space waypoints for MPC demonstration."""

    def __init__(self):
        super().__init__('joint_space_mpc_demo')

        self.declare_parameter('waypoint_hold_time', 5.0)
        self.hold_time = self.get_parameter('waypoint_hold_time').value

        self.target_pub = self.create_publisher(
            JointState, '/space_arm/target_joint_state', 10
        )

        self.waypoint_idx = 0
        self.timer = self.create_timer(self.hold_time, self._next_waypoint)

        # Publish first waypoint immediately
        self._publish_waypoint(0)
        self.get_logger().info('Joint-space MPC demo started')

    def _publish_waypoint(self, idx: int):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = WAYPOINTS[idx]
        self.target_pub.publish(msg)
        self.get_logger().info(
            f'Waypoint {idx}: {[f"{v:.2f}" for v in WAYPOINTS[idx]]}'
        )

    def _next_waypoint(self):
        self.waypoint_idx = (self.waypoint_idx + 1) % len(WAYPOINTS)
        self._publish_waypoint(self.waypoint_idx)


def main(args=None):
    rclpy.init(args=args)
    node = JointSpaceMPCDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
