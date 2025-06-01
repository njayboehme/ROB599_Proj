#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
import google
print(dir(google))
from google import genai
from vlm_nav_msgs.srv import Text
from vlm_nav_msgs.srv import ImgPath


API_KEY = 'AIzaSyCW_VwkFmYB1MzwaII0eTalkGt65nH8g7M'

class VLM(Node):
    def __init__(self):
        super().__init__('VLM')
        # TODO: Need to publish the goal pose to the right topic.
        # Might be better as a service if possible.
        self.pub_goal = self.create_publisher(Pose2D, 'TODO_GOAL', 10)
        self.timer_goal = self.create_timer(1, self.pub_goal_callback)
        self.img_srv = self.create_service(ImgPath, 'vlm_img', self.img_srv_callback)
        self.text_srv = self.create_service(Text, 'vlm_text', self.text_srv_callback)
        self.gemini_client = genai.Client(api_key=API_KEY)
        self.text_inp = None
        self.img = None

    def pub_goal_callback(self):
        txt = self.run_vlm()
        if txt is not None:
            msg = Pose2D()
            # TODO: do some processing and get into goal pose
            self.pub_goal.publish(msg)
        

    def run_vlm(self):
        if self.text_inp != None and self.img != None:
            out = self.gemini_client.models.generate_content(
                model="gemini-2.0-flash", contents=[self.text_inp, self.img]
            )
            self.text_inp = None
            self.img = None
            self.get_logger().info(f'{out.text}')

            return out.text
        return None
        

    def text_srv_callback(self, request, response):
        print(f"got request: {request.inp}")
        self.text_inp = request.inp
        return response
    
    def img_srv_callback(self, request, response):
        print(f"got img request: {request.path}")
        print(type(request.path))
        self.img = self.gemini_client.files.upload(file=request.path)
        return response


def main(args=None):
	rclpy.init(args=args)
	publisher = VLM()

	rclpy.spin(publisher)

	rclpy.shutdown()

if __name__ == '__main__':
	main()