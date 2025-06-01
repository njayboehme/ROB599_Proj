#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vlm_nav_msgs.srv import BasePromptService
from vlm_nav_msgs.msg import BasePrompt
import google
from google import genai

API_KEY = ' '


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

        # Subscribe to /task_feedback so we can use feedback as new prompt
        self._feedback_sub = self.create_subscription(
            String,
            'task_feedback',
            self.task_feedback_callback,
            10
        )

        # Gemini client
        self.gemini_client = genai.Client(api_key=API_KEY)

        # Stores the last text prompt (either initial or feedback)
        self.text_inp = None
        # Stores the last image, once uploaded
        self.img = None
        # Flag that feedback prompt has been received
        self.feedback_prompt_received = False
        # Flag that base prompt has been received
        self.base_prompt_received = False

        self.get_logger().info("VLM node ready; waiting for BasePromptService requests or task_feedback.")

    def base_prompt_callback(self, request, response):
        """
        Called whenever someone calls /base_prompt with a BasePrompt.
        We immediately store the text + image, then ask Gemini for waypoints.
        """
        prompt: BasePrompt = request.prompt
        text = prompt.text.strip()
        image_path = prompt.image_path.strip()

        self.get_logger().info(f"Received BasePrompt:\n  text: {text}\n  image_path: {image_path}")

        # Store text, upload image
        self.text_inp = text
        try:
            self.img = self.gemini_client.files.upload(file=image_path)
        except Exception as e:
            self.get_logger().error(f"Failed to upload image '{image_path}': {e}")
            response.success = False
            return response

        # Flag that base prompt has been received
        self.base_prompt_received = True

        # Immediately call try_publish with initial prompt
        self.try_publish()

        response.success = True
        return response

    def task_feedback_callback(self, msg: String):
        """
        Called whenever problem_manager publishes on /task_feedback.
        We take that JSON text as a new prompt and run try_publish() again.
        """
        feedback_text = msg.data.strip()
        self.get_logger().info(f"Received task_feedback as new prompt: {feedback_text!r}")

        # Store the received feedback as the new text_inp
        self.text_inp = "After executing your plan, you received this feedback: " + feedback_text + " Use it to replan the waypoints."

        # Flag that feedback prompt has been received
        self.feedback_prompt_received = True

        # Call try_publish, re-querying Gemini with the feedback as prompt
        self.try_publish()

    def try_publish(self):
        """
        If both text_inp and img are present, call Gemini and ask the user
        whether to publish the resulting waypoint string. If the user replies 'y',
        publish once and then reset text_inp & img to None so we only publish once per iteration.
        """
        if self.base_prompt_received or self.feedback_prompt_received:
            out = self.gemini_client.models.generate_content(
                model="gemini-2.0-flash",
                contents=[self.text_inp, self.img]
            )
            waypoint_str = out.text.strip()
            self.get_logger().info(f"VLM ▶ {waypoint_str}")

            # Ask the user before publishing
            try:
                ans = input(f"Publish these waypoints? {waypoint_str}  (y/n): ").strip().lower()
            except EOFError:
                ans = 'n'

            if ans == 'y':
                msg = String()
                msg.data = waypoint_str
                self.vlm_waypoint_pub.publish(msg)
                self.get_logger().info(f"Published VLM waypoints string: {waypoint_str}")
                # Reset so we only publish once per new prompt
                self.text_inp = None
                # self.img = None
                self.feedback_prompt_received = False
                self.base_prompt_received = False
            else:
                self.get_logger().info("User chose not to publish. Keeping inputs for later.")


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
