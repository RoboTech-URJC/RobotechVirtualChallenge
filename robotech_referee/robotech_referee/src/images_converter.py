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
        self.purple_img_ = None
        self.red_img_ = None
        self.football_img_ = None
        self.purple_received_ = False
        self.red_received_ = False
        self.football_received_ = False

        self.purple_img_sub_ = rospy.Subscriber('/purple/camera/rgb/image_raw', Image, self.purpleCb_)
        self.red_img_sub_ = rospy.Subscriber('/red/camera/rgb/image_raw', Image, self.redCb_)
        self.football_img_sub_ = rospy.Subscriber('/camera_1/camera/image_raw', Image, self.footballCb_)

        self.purple_img_pub_ = rospy.Publisher('/robotech_vc_hub/purple_img', String, queue_size=1)
        self.red_img_pub_ = rospy.Publisher('/robotech_vc_hub/red_img', String, queue_size=1)
        self.football_img_pub_ = rospy.Publisher('/robotech_vc_hub/football_img', String, queue_size=1)


    def step(self):
        s = String()

        if (self.purple_received_):
            cv_img1 = self.getCvImg_(self.purple_img_)

            _, buffer = cv2.imencode('.jpg', cv_img1)
            image_as_str1 = base64.b64encode(buffer).decode('utf-8')
            s.data = image_as_str1
            self.purple_img_pub_.publish(s)

        if (self.red_received_):
            cv_img2 = self.getCvImg_(self.red_img_)

            _, buffer = cv2.imencode('.jpg', cv_img2)
            image_as_str2 = base64.b64encode(buffer).decode('utf-8')
            s.data = image_as_str2
            self.red_img_pub_.publish(s)

        if (self.football_received_):
            cv_img3 = self.getCvImg_(self.football_img_)

            _, buffer = cv2.imencode('.jpg', cv_img3)
            image_as_str3 = base64.b64encode(buffer).decode('utf-8')
            s.data = image_as_str3
            self.football_img_pub_.publish(s)

        self.purple_received_ = False
        self.red_received_ = False
        self.football_received_ = False

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

    def redCb_(self, msg):
        if (not self.red_received_):
            self.red_received_ = True

        self.red_img_ = msg

    def footballCb_(self, msg):
        if (not self.football_received_):
            self.football_received_ = True

        self.football_img_ = msg


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
