#!/usr/bin/env python3
"""
MPC Controller Node for Space Robotic Arm.

Reads joint states, computes optimal torque commands using CasADi-based MPC,
and publishes effort commands. Designed for microgravity operation where
traditional gravity compensation is unnecessary but precise control is critical.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

from space_arm_control.mpc_solver import MPCSolver


class MPCControllerNode(Node):
    """ROS2 node wrapping the MPC solver for real-time arm control."""

    def __init__(self):
        super().__init__('mpc_controller')

        # Declare and read parameters
        self._declare_parameters()
        self._read_parameters()

        # Initialize MPC solver
        self.solver = MPCSolver(
            num_joints=self.num_joints,
            prediction_horizon=self.prediction_horizon,
            control_horizon=self.control_horizon,
            dt=self.dt,
            torque_limits=self.torque_limits,
            velocity_limits=self.velocity_limits,
            state_weight=self.state_weight,
            control_weight=self.control_weight,
            terminal_weight=self.terminal_weight,
        )
        self.get_logger().info('MPC solver initialized')

        # State variables
        self.current_positions = np.zeros(self.num_joints)
        self.current_velocities = np.zeros(self.num_joints)
        self.target_positions = np.zeros(self.num_joints)
        self.joint_state_received = False

        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_state_callback, 10
        )
        self.target_sub = self.create_subscription(
            JointState, '/space_arm/target_joint_state', self._target_callback, 10
        )
        self.ee_target_sub = self.create_subscription(
            PoseStamped, '/space_arm/target_ee_pose', self._ee_target_callback, 10
        )

        # Publisher for effort commands
        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/effort_controller/commands', 10
        )

        # Control loop timer
        control_period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(control_period, self._control_loop)

        self.get_logger().info(
            f'MPC controller started at {self.control_rate} Hz '
            f'(N={self.prediction_horizon}, dt={self.dt})'
        )

    def _declare_parameters(self):
        self.declare_parameter('prediction_horizon', 20)
        self.declare_parameter('control_horizon', 10)
        self.declare_parameter('dt', 0.05)
        self.declare_parameter('num_joints', 6)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('joint_names', [
            'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'
        ])
        self.declare_parameter('state_weight', [10.0] * 6)
        self.declare_parameter('velocity_weight', [1.0] * 6)
        self.declare_parameter('control_weight', [0.01] * 6)
        self.declare_parameter('terminal_weight', [50.0] * 6)
        self.declare_parameter('torque_limits', [100.0, 100.0, 100.0, 50.0, 50.0, 30.0])
        self.declare_parameter('velocity_limits', [1.0, 1.0, 1.0, 1.5, 1.5, 2.0])

    def _read_parameters(self):
        self.prediction_horizon = self.get_parameter('prediction_horizon').value
        self.control_horizon = self.get_parameter('control_horizon').value
        self.dt = self.get_parameter('dt').value
        self.num_joints = self.get_parameter('num_joints').value
        self.control_rate = self.get_parameter('control_rate').value
        self.joint_names = self.get_parameter('joint_names').value
        self.state_weight = np.array(self.get_parameter('state_weight').value)
        self.velocity_weight = np.array(self.get_parameter('velocity_weight').value)
        self.control_weight = np.array(self.get_parameter('control_weight').value)
        self.terminal_weight = np.array(self.get_parameter('terminal_weight').value)
        self.torque_limits = np.array(self.get_parameter('torque_limits').value)
        self.velocity_limits = np.array(self.get_parameter('velocity_limits').value)

    def _joint_state_callback(self, msg: JointState):
        """Update current joint state from the joint_state_broadcaster."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
                if msg.velocity:
                    self.current_velocities[i] = msg.velocity[idx]
        self.joint_state_received = True

    def _target_callback(self, msg: JointState):
        """Update target joint positions."""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.target_positions[i] = msg.position[idx]
        self.get_logger().debug(f'New target: {self.target_positions}')

    def _ee_target_callback(self, msg: PoseStamped):
        """Handle end-effector pose target (requires IK -- placeholder)."""
        # TODO: Implement inverse kinematics to convert EE pose to joint targets.
        # For now, log the received target.
        self.get_logger().info(
            f'EE target received at [{msg.pose.position.x:.3f}, '
            f'{msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}] '
            '-- IK not yet implemented, ignoring.'
        )

    def _control_loop(self):
        """Run one MPC iteration and publish torque commands."""
        if not self.joint_state_received:
            return

        # Current state: [positions; velocities]
        x0 = np.concatenate([self.current_positions, self.current_velocities])
        x_ref = np.concatenate([self.target_positions, np.zeros(self.num_joints)])

        # Solve MPC
        torques, solve_info = self.solver.solve(x0, x_ref)

        if torques is None:
            self.get_logger().warn(
                f'MPC solve failed: {solve_info.get("status", "unknown")}',
                throttle_duration_sec=2.0,
            )
            # Publish zero torques as safety fallback
            torques = np.zeros(self.num_joints)

        # Publish effort commands
        msg = Float64MultiArray()
        msg.data = torques.tolist()
        self.effort_pub.publish(msg)

    def destroy_node(self):
        """Clean shutdown."""
        # Send zero torques before shutting down
        msg = Float64MultiArray()
        msg.data = [0.0] * self.num_joints
        self.effort_pub.publish(msg)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MPCControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
