#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from proj_msgs.srv import ImgPath
from cv_bridge import CvBridge
import cv2
import os


class CameraNode(Node):
    def __init__(self):
        super().__init__('img_client')
        # TODO: Add in the camera subscription
        # self.camera_sub = self.create_subscription(Image, 'TODO', self.sub_camera_callback, 10)
        # self.img = None
        self.b = CvBridge()

        # TESTING
        test_img_path = '/home/nboehme/Desktop/ROB599_Proj/proj_ws/src/project/test_img.jpg'
        cv_img = cv2.imread(test_img_path)
        print(type(cv_img))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'meh'
        self.img = self.b.cv2_to_imgmsg(cv_img, header=header)
        # TESTING

        self.filename = 'ros_img'
        self.client = self.create_client(ImgPath, 'vlm_img')
        while not self.client.wait_for_service(timeout_sec=1):
            self.get_logger().info('waiting for service to start')

    def send_request(self):
        if self.img is not None:
            self.get_logger().info('Sending image')
            request = ImgPath.Request()
            cv2.imwrite(self.filename + '.png', self.b.imgmsg_to_cv2(self.img))
            path_to_img = os.path.abspath(self.filename + '.png')
            self.get_logger().info(f'Image path {path_to_img}')
            
            request.path = path_to_img
            self.response = self.client.call_async(request)
        else:
            self.get_logger().info('No image to send')
    
    def sub_camera_callback(self, msg):
        self.img = msg


# This is the entry point of the node for a set of single calls.
def main(args=None):	
    rclpy.init(args=args)
    client = CameraNode()

    while True:
    # Make the asynchronous call.
        inp = input("DataSender: Enter y to take an image. Enter q to quit. ")
        if inp.lower() == 'y':
            client.send_request()
        elif inp.lower() == 'q':
            break

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
