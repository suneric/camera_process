#!/usr/bin/env python

"""
republish image with compressed
reference:
https://gist.github.com/awesomebytes/958a5ef9e63821a28dc05775840c34d9
"""

import rospy
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as py
import os

class ImageProcess:
    def __init__(self):
        self.depthSub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self._depth_callback)
        self.colorSub = rospy.Subscriber('/camera/color/image_raw', Image, self._color_callback)
        self.cvbr = CvBridge()
        self.colorComp = []
        self.depthComp = []

    def _color_callback(self,image):
        try:
            colorCV = self.cvbr.imgmsg_to_cv2(image, "bgr8")
            self.colorComp = self.cvbr.cv2_to_compressed_imgmsg(colorCV,"jpg")
            self.colorComp.header = image.header
        except CvBridgeError as e:
            print(e)

    def _depth_callback(self,image):
        try:
            depthCV = self.cvbr.imgmsg_to_cv2(image, "mono16")
            data = self.cvbr.cv2_to_compressed_imgmsg(depthCV,"png")
            self.depthComp.header = image.header
            self.depthComp.format = '16UC1; compressDepth'
            self.depthComp.data = "\x00\x00\x00\x00\x88\x9c\x5c\xaa\x00\x40\x4b\xb7"+data
        except CvBridgeError as e:
            print(e)

def start(dim,fps):
    # init ros node
    rospy.init_node("realsense_d435", anonymous=True)
    colorPub = rospy.Publisher("camera/color/compressed", CompressedImage, queue_size=3)
    depthPub = rospy.Publisher("camera/depth/compressed", CompressedImage, queue_size=3)
    ip = ImageProcess()
    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        if len(ip.colorComp) > 0:
            colorPub.publish(ip.colorComp)
        if len(ip.depthComp > 0):
            depthPub.publish(ip.depthComp)


if __name__ == '__main__':
    dim = (640,480)
    fps = 15
    start(dim,fps)
