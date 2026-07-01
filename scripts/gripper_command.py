#!/usr/bin/env python3
"""Send a single parallel gripper command goal."""

from __future__ import annotations

import sys
from typing import Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import ParallelGripperCommand


class GripperCommandNode(Node):
    """Minimal action client for the parallel gripper controller."""

    def __init__(self) -> None:
        """Initialize the gripper action client node."""
        super().__init__('gripper_command_node')
        self.declare_parameter('position', 0.0)
        self.declare_parameter('max_vel', -1.0)
        self.declare_parameter('max_effort', -1.0)
        self.declare_parameter('verbose', True)
        self.declare_parameter(
            'action_name',
            '/gripper_controller/gripper_cmd',
        )
        self._client = ActionClient(
            self,
            ParallelGripperCommand,
            self.get_parameter('action_name').value,
        )

    def send_goal(self) -> None:
        """Wait for the action server and send one goal."""
        verbose = bool(self.get_parameter('verbose').value)
        action_name = self.get_parameter('action_name').value
        position = float(self.get_parameter('position').value or 0.0)
        max_vel = float(self.get_parameter('max_vel').value or 0.0)
        max_effort = float(self.get_parameter('max_effort').value or 0.0)
        velocity = [] if max_vel < 0.0 else [max_vel]
        effort = [] if max_effort < 0.0 else [max_effort]
        max_vel_log = 'NONE' if max_vel < 0.0 else str(max_vel)
        max_effort_log = 'NONE' if max_effort < 0.0 else str(max_effort)

        if verbose:
            self.get_logger().info(
                f'Sending gripper goal to {action_name}: '
                f'position={position} max_vel={max_vel_log} '
                f'max_effort={max_effort_log}'
            )

        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Gripper action server not available.')
            raise RuntimeError('Gripper action server not available')

        goal = ParallelGripperCommand.Goal()
        goal.command.name = ['gripper']
        goal.command.position = [position]
        goal.command.velocity = velocity
        goal.command.effort = effort

        if verbose:
            self.get_logger().info('Waiting for gripper goal response...')

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Gripper goal rejected.')
            raise RuntimeError('Gripper goal rejected')

        if verbose:
            self.get_logger().info(
                'Gripper goal accepted, waiting for result...'
            )

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        self.get_logger().info(
            f'Gripper goal finished: stalled={result.result.stalled} '
            f'reached_goal={result.result.reached_goal}'
        )


def main(argv: Sequence[str] | None = None) -> int:
    """Entrypoint for the gripper command executable."""
    rclpy.init(args=list(argv) if argv is not None else None)
    node = GripperCommandNode()
    try:
        node.send_goal()
    except Exception as exc:  # pragma: no cover - surfaced via ROS logging
        node.get_logger().error(str(exc))
        node.destroy_node()
        rclpy.shutdown()
        return 1

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
