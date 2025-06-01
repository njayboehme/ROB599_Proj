#!/usr/bin/env python3
"""
problem_manager.py

A ROS 2 Python node that:
  • Subscribes to /vlm_waypoints (std_msgs/String).
  • Parses the incoming JSON‐style array of waypoints (possibly wrapped in markdown fences).
  • Converts to geometry_msgs/Point list and sends to nav_game via an ActionClient.
  • Prints every feedback message (the growing trajectory).
  • Prints the final result (successful_waypoints).
  • Shuts down once the result is received.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.action import ActionClient
from nav_game_msgs.action import Waypoints
from geometry_msgs.msg import Point
import re
import ast


class ProblemManager(Node):
    def __init__(self):
        super().__init__('problem_manager')

        # Create the ActionClient for 'waypoints'
        self._client = ActionClient(self, Waypoints, 'waypoints')

        # Subscribe to /vlm_waypoints (std_msgs/String)
        self._vlm_sub = self.create_subscription(
            String,
            'vlm_waypoints',
            self.vlm_callback,
            10
        )

        # Will hold the current goal handle
        self._goal_handle = None

        self.get_logger().info('Waiting for /vlm_waypoints messages to parse and send to nav_game...')

        # Wait for the action server to come up (do not send any goal until we receive waypoints)
        self._client.wait_for_server()
        self.get_logger().info('Action server /waypoints is available.')

    def vlm_callback(self, msg: String):
        """
        Called whenever a new String is published on /vlm_waypoints.
        The payload may include markdown fences (```json ... ```) surrounding the JSON array.
        Extract the JSON array, parse it into a Python list of [x,y] pairs, convert to Points,
        and send as an action goal to nav_game.
        """
        raw = msg.data
        self.get_logger().info(f"Received raw /vlm_waypoints: {raw!r}")

        # Use a regex to extract the first substring that looks like [ ... ] (including nested brackets)
        match = re.search(r'\[.*\]', raw, re.DOTALL)
        if not match:
            self.get_logger().error('Could not find a JSON array in the received string.')
            return

        array_str = match.group(0)
        self.get_logger().info(f"Extracted JSON‐style array: {array_str}")

        # Safely evaluate the array_str to a Python list
        try:
            parsed = ast.literal_eval(array_str)
        except Exception as e:
            self.get_logger().error(f'Failed to parse array string: {e}')
            return

        # Verify parsed is a list of [x,y] lists or tuples
        if not isinstance(parsed, list) or not all(
            isinstance(pt, (list, tuple)) and len(pt) == 2 for pt in parsed
        ):
            self.get_logger().error('Parsed data is not a list of 2‐element lists/tuples.')
            return

        # Convert to a list of (int,int) tuples
        waypoint_list = [(int(pt[0]), int(pt[1])) for pt in parsed]
        self.get_logger().info(f'Parsed waypoints: {waypoint_list}')

        # Now send that as a goal
        self.send_goal(waypoint_list)

    def send_goal(self, waypoint_list):
        """
        Build a Waypoints.Goal from the given list of (col,row) tuples,
        send the goal asynchronously, and register feedback & result callbacks.
        """
        goal_msg = Waypoints.Goal()
        for (c, r) in waypoint_list:
            pt = Point()
            pt.x = float(c)
            pt.y = float(r)
            pt.z = 0.0
            goal_msg.waypoints.append(pt)

        self.get_logger().info(f'Sending goal with {len(waypoint_list)} waypoint(s) to /waypoints...')
        send_goal_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """
        Called when the action server accepts or rejects our goal.
        If accepted, request the result.
        """
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was rejected by nav_game action server.')
            return

        self.get_logger().info('Goal accepted by nav_game. Awaiting result...')
        get_result_future = self._goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """
        Called whenever the action server publishes feedback.
        Print the current trajectory.
        """
        trajectory = feedback_msg.feedback.trajectory
        simple_list = [(int(pt.x), int(pt.y)) for pt in trajectory]
        self.get_logger().info(f'[FEEDBACK] trajectory so far: {simple_list}')

    def get_result_callback(self, future):
        """
        Called once when nav_game sets the final result.
        Print the successful_waypoints and then shut down.
        """
        result = future.result().result
        success_list = [(int(pt.x), int(pt.y)) for pt in result.successful_waypoints]
        self.get_logger().info(f'[RESULT] successful_waypoints: {success_list}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ProblemManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
