#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import google
from google import genai
from vlm_nav_msgs.srv import Text, ImgPath

API_KEY = 'xyz'

class VLM(Node):
    def __init__(self):
        super().__init__('VLM')

        # Publisher for the waypoint‐string (only when ready)
        self.vlm_waypoint_pub = self.create_publisher(String, 'vlm_waypoints', 10)

        # Services to receive text and image inputs
        self.text_srv = self.create_service(Text, 'vlm_text', self.text_srv_callback)
        self.img_srv = self.create_service(ImgPath, 'vlm_img', self.img_srv_callback)

        self.gemini_client = genai.Client(api_key=API_KEY)
        self.text_inp = None
        self.img = None

        self.get_logger().info("VLM node ready; waiting for text+image services.")

    def try_publish(self):
        """
        If both text_inp and img are present, call Gemini and publish the result once.
        Then reset text_inp and img to None.
        """
        if self.text_inp is not None and self.img is not None:
            out = self.gemini_client.models.generate_content(
                model="gemini-2.0-flash",
                contents=[self.text_inp, self.img]
            )
            waypoint_str = out.text.strip()  # e.g. "[(2,0),(2,3),(5,3),(7,7)]"
            self.get_logger().info(f"VLM ▶ {waypoint_str}")

            msg = String()
            msg.data = waypoint_str
            self.vlm_waypoint_pub.publish(msg)
            self.get_logger().info(f"Published VLM waypoints string: {waypoint_str}")

            # Reset so we only publish once per new pair
            self.text_inp = None
            self.img = None

    def text_srv_callback(self, request, response):
        """
        Called when a client sends a Text request.
        Store request.inp, then attempt to publish if image already exists.
        """
        self.get_logger().info(f"Received text input: {request.inp}")
        self.text_inp = request.inp
        self.try_publish()
        return response
    
    def img_srv_callback(self, request, response):
        """
        Called when a client sends an ImgPath request.
        Upload the image, then attempt to publish if text already exists.
        """
        self.get_logger().info(f"Received image path: {request.path}")
        self.img = self.gemini_client.files.upload(file=request.path)
        self.try_publish()
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
