#!/usr/bin/env python3
"""
problem_manager.py

A ROS 2 Python node that:
  • Creates an ActionClient for nav_game_msgs/Waypoints.
  • Waits for the nav_game action server (/waypoints) to be available.
  • Sends a hardcoded list of geometry_msgs/Point waypoints.
  • Prints every feedback message (the growing trajectory).
  • Prints the final result (successful_waypoints).
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_game_msgs.action import Waypoints
from geometry_msgs.msg import Point


class ProblemManager(Node):
    def __init__(self):
        super().__init__('problem_manager')

        # Create the ActionClient, pointing to the 'waypoints' action name
        self._client = ActionClient(self, Waypoints, 'waypoints')

        # Hardcoded list of waypoints (col, row) to send:
        # • You can modify these before sending.
        self._waypoint_list = [
            (3, 0),
            (3, 4),
            (7, 7),
        ]

        # Will hold the current goal handle
        self._goal_handle = None

        # Once we are up, immediately try to send a goal
        self.get_logger().info('Waiting for action server to become available...')
        self._client.wait_for_server()
        self.get_logger().info('Action server available. Sending goal...')
        self.send_goal()

    def send_goal(self):
        """
        Build a Waypoints.Goal from self._waypoint_list,
        send the goal, register feedback & result callbacks.
        """
        # Construct the Goal message
        goal_msg = Waypoints.Goal()
        for (c, r) in self._waypoint_list:
            pt = Point()
            pt.x = float(c)
            pt.y = float(r)
            pt.z = 0.0
            goal_msg.waypoints.append(pt)

        # Send the goal asynchronously, storing the future
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Called when the action server has accepted (or rejected) our goal.
        If accepted, future.result() is a GoalHandle that we store and can later use to get the result.
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected by the action server.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        # Request the result, returns a future
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Called whenever the action server publishes feedback.
        feedback_msg is of type Waypoints.Feedback, with a field `trajectory: List[Point]`.
        We simply print the entire trajectory array so far.
        """
        trajectory = feedback_msg.feedback.trajectory
        # Convert Points → simple tuples for logging
        simple_list = [(int(pt.x), int(pt.y)) for pt in trajectory]
        self.get_logger().info(f'[FEEDBACK] trajectory so far: {simple_list}')

    def get_result_callback(self, future):
        """
        Called once when the action server sets the final result.
        future.result().result is a Waypoints.Result message,
        containing `successful_waypoints: List[Point]`.
        """
        result = future.result().result
        success_list = [(int(pt.x), int(pt.y)) for pt in result.successful_waypoints]
        self.get_logger().info(f'[RESULT] successful_waypoints: {success_list}')
        # Once we get the result, we can exit.
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ProblemManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean teardown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
