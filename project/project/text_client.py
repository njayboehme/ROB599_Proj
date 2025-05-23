#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from proj_msgs.srv import Text


class TextNode(Node):
	def __init__(self):
		super().__init__('text_client')
		self.client = self.create_client(Text, 'vlm_text')

		while not self.client.wait_for_service(timeout_sec=1):
			self.get_logger().info('waiting for service to start')

	def send_request(self, text_inp):
		request = Text.Request()
		request.inp = text_inp
		self.response = self.client.call_async(request)


# This is the entry point of the node for a set of single calls.
def main(args=None):	
    rclpy.init(args=args)
    client = TextNode()

    while True:
    # Make the asynchronous call.
        inp = input("DataSender: Enter text to send to the VLM. Enter q to quit. ")
        if inp.lower() == 'q':
            break
        client.send_request(inp)

        while rclpy.ok():
            rclpy.spin_once(client)
            if client.response.done():
                try:
                    answer = client.response.result()
                except Exception as e:
                    client.get_logger().info(f'Service call failed: {e}')
                else:
                    break

    rclpy.shutdown()


# This is the entry point if we run this node as an executable.
if __name__ == '__main__':
	main()
