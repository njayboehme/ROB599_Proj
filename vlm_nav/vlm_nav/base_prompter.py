#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from vlm_nav_msgs.msg import BasePrompt
from vlm_nav_msgs.srv import BasePromptService

class BasePrompter(Node):
    def __init__(self):
        super().__init__('base_prompter')

        # Create a client for the BasePromptService
        self._client = self.create_client(BasePromptService, 'base_prompt')
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for base_prompt service…')

        self.get_logger().info("base_prompter started. Calling BasePromptService with initial prompt.")

        # Send the base prompt once at startup
        self.call_base_prompt()

    def call_base_prompt(self):
        """
        Build a BasePrompt request and call the service.
        """
        # Hardcoded text and image_path for this example
        text = (
            "You are a 2D gridworld path planner. Your job is to plan a path from your current "
            "position (Blue) to the target location (Red). Free cells are white, while obstacles "
            "are black cells. The path you generate must be composed of waypoints that define the "
            "path that you will take to the target. You must avoid obstacles. Output exactly one "
            "JSON array of integer pairs, no explanatory text. After you send that, the problem_manager "
            "will attempt to execute it, then publish back a “feedback” message containing the sequence "
            "of cells actually traversed and which waypoints succeeded. Using that information you must "
            "understand where the path failed. You must then produce a new JSON list that corrects any "
            "collisions or misalignments. Repeat until you reach the goal."
        )
        image_path = '/home/raghav/rob599/vlm_nav_ws/scene.png'

        # Create the service request
        request = BasePromptService.Request()
        request.prompt = BasePrompt(text=text, image_path=image_path)

        # Call the service
        future = self._client.call_async(request)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """
        Handle the service response.
        """
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            rclpy.shutdown()
            return

        if response.success:
            self.get_logger().info("BasePrompt was successfully sent to VLM.")
        else:
            self.get_logger().error("VLM failed to process the base prompt.")

        # Optionally exit after sending the base prompt
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = BasePrompter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass  # Allow clean exit on Ctrl+C
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
