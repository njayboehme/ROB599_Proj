#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vlm_nav_msgs.msg import BasePrompt
from std_msgs.msg import String

class BasePrompter(Node):
    def __init__(self):
        super().__init__('base_prompter')

        # Publisher that will emit BasePrompt messages on 'base_prompt'
        self._pub = self.create_publisher(BasePrompt, 'base_prompt', 10)

        self.get_logger().info("base_prompter started. Ready to send BasePrompt messages.")

        # Immediately prompt the user once at startup
        self.send_base_prompt()

    def send_base_prompt(self):
        """
        Read user input for the 'text' and 'image_path', then publish a BasePrompt.
        """
        # Read the free‐form text prompt from stdin
        text = input("Enter your high‐level prompt text: ").strip()

        # Read the image path from stdin
        image_path = input("Enter the map image file path: ").strip()

        # Build and publish the BasePrompt msg
        msg = BasePrompt()
        msg.text = text
        msg.image_path = image_path

        self._pub.publish(msg)
        self.get_logger().info(f"Published BasePrompt:\n  text: {msg.text}\n  image_path: {msg.image_path}")

def main(args=None):
    rclpy.init(args=args)
    node = BasePrompter()
    try:
        # Keep spinning in case you want to extend this later
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
