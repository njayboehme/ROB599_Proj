#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vlm_nav_msgs.srv import BasePromptService
from vlm_nav_msgs.msg import BasePrompt
import google
from google import genai

API_KEY = ''


class VLM(Node):
    def __init__(self):
        super().__init__('VLM')

        # Publisher for the waypoint‐string (only when approved by user)
        self.vlm_waypoint_pub = self.create_publisher(String, 'vlm_waypoints', 10)

        # Single service that receives a BasePrompt (text + image_path)
        self.base_prompt_srv = self.create_service(
            BasePromptService,
            'base_prompt',
            self.base_prompt_callback
        )

        # Gemini client
        self.gemini_client = genai.Client(api_key=API_KEY)

        self.get_logger().info("VLM node ready; waiting for BasePromptService requests.")

    def base_prompt_callback(self, request, response):
        """
        Called whenever someone calls /base_prompt with a BasePrompt.
        We immediately send the text+image to Gemini, then ask the user whether to publish.
        Return success=True if published, False otherwise.
        """
        prompt: BasePrompt = request.prompt
        text = prompt.text.strip()
        image_path = prompt.image_path.strip()

        self.get_logger().info(f"Received BasePrompt:\n  text: {text}\n  image_path: {image_path}")

        # 1) Upload image to Gemini
        try:
            gen_image = self.gemini_client.files.upload(file=image_path)
        except Exception as e:
            self.get_logger().error(f"Failed to upload image '{image_path}': {e}")
            response.success = False
            return response

        # 2) Query Gemini with [text, gen_image]
        try:
            out = self.gemini_client.models.generate_content(
                model="gemini-2.0-flash",
                contents=[text, gen_image]
            )
        except Exception as e:
            self.get_logger().error(f"Gemini generate_content error: {e}")
            response.success = False
            return response

        waypoint_str = out.text.strip()  # e.g. "[(2,0),(2,3),(5,3),(7,7)]"
        self.get_logger().info(f"VLM ▶ {waypoint_str}")

        # 3) Ask the user whether to publish
        try:
            ans = input(f"Publish these waypoints? {waypoint_str}  (y/n): ").strip().lower()
        except EOFError:
            ans = 'n'

        if ans == 'y':
            msg = String()
            msg.data = waypoint_str
            self.vlm_waypoint_pub.publish(msg)
            self.get_logger().info(f"Published VLM waypoints string: {waypoint_str}")
            response.success = True
        else:
            self.get_logger().info("User chose not to publish.")
            response.success = False

        return response


def main(args=None):
    rclpy.init(args=args)
    node = VLM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
