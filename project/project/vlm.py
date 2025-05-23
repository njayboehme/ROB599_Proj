# subscribes to some overhead image of the rviz environment. 
# subscribes to some node that gets text from the user
# publishes goal node to navigate to
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# from std_msgs.msg import String
import google
print(dir(google))
from google import genai
from proj_msgs.srv import Text


API_KEY = 'AIzaSyDAKdFtnAKaHSKAjeKSXytiRdvRbEx_qsY'

class VLM(Node):
    def __init__(self):
        super().__init__('VLM')
        # self.pub_goal = self.create_publisher(Float32, 'raw_latency', 10)
        # self.timer_goal = self.create_timer(1, self.pub_goal_callback)
        self.text_srv = self.create_service(Text, 'vlm_text', self.text_srv_callback)
        self.gemini_client = genai.Client(api_key=API_KEY)
        self.text_inp = None
        self.img = None

    def pub_goal_callback(self):
        txt = self.run_vlm()
        # TODO: do some processing and get into goal pose

    def run_vlm(self):
        if self.text_inp != None and self.img != None:
            out = self.gemini_client.models.generate_content(
                model="gemini-2.5-flash", contents=[self.text_inp, self.img]
            )
            self.text_inp = None
            self.img = None
            return out.text
        

    def text_srv_callback(self, request, response):
        print(f"got request: {request.inp}")
        self.text_inp = request.inp
        # out = self.gemini_client.models.generate_content(
        #     model="gemini-2.5-flash", contents=request.inp
        # )
        # print(out.text)
        return response
    
    def img_srv_callback(self, request, response):
        self.img = self.gemini_client.files.upload(file=request.path)
        return response


def main(args=None):
	rclpy.init(args=args)
	publisher = VLM()

	rclpy.spin(publisher)

	rclpy.shutdown()

if __name__ == '__main__':
	main()