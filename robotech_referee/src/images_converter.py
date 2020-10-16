#! /usr/bin/env python3

'''
Get the images published by the two robots and converts then to base64
'''

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

import cv2
import base64
import sys

class ImageConverter():
    def __init__(self):
        self.bridge = CvBridge()
        self.purple_img_ = self.red_img_ = None
        self.purple_received_ = self.red_received_ = False

        self.purple_img_sub_ = rospy.Subscriber('/purple/camera/rgb/image_raw', Image, self.purpleCb_)
        
        self.red_img_sub_ = rospy.Subscriber('/red/camera/rgb/image_raw', Image, self.redCb_)

        self.purple_img_pub_ = rospy.Publisher('/robotech_vc_hub/purple_img', String, queue_size=10)
        self.red_img_pub_ = rospy.Publisher('/robotech_vc_hub/red_img', String, queue_size=1)


    def step(self):
        purple_cv = red_img = None

        s = String()

        if (self.purple_received_):
            purple_cv = self.getCvImg_(self.purple_img_)

            _, buffer = cv2.imencode('.jpg', purple_cv)
            image_as_str = base64.b64encode(buffer).decode('utf-8')
            s.data = image_as_str
            self.purple_img_pub_.publish(s)

        if (self.red_received_):
            red_cv = self.getCvImg_(self.red_img_)

            _, buffer = cv2.imencode('.jpg', red_cv)
            image_as_str = base64.b64encode(buffer).decode('utf-8')
            s.data = image_as_str
            self.red_img_pub_.publish(s)

        print("Step!")

    def getCvImg_(self, inp_img):
        try:
            out_cv = self.bridge.imgmsg_to_cv2(inp_img, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        return out_cv


    def purpleCb_(self, msg):
        if (not self.purple_received_):
            self.purple_received_ = True

        self.purple_img_ = msg

        print("Purple Img Received!")

    def redCb_(self, msg):
        if (not self.red_received_):
            self.red_received_ = True

        self.red_img_ = msg

        print("Red Img Received!")


def main(args):
    rospy.init_node('images_converter_node')
    loop_rate = rospy.Rate(4)

    try:
        converter = ImageConverter()
        while (not rospy.is_shutdown()):
            converter.step()
            loop_rate.sleep()

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":

    main(sys.argv)
