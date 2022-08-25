#!/usr/bin/env python3

"""
republish image with compressed
reference:
https://gist.github.com/awesomebytes/958a5ef9e63821a28dc05775840c34d9
"""
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
import numpy as np

class RSImageCompressor:
    def __init__(self):
        self.cvbr = CvBridge()
        self.depthSub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self._depth_callback)
        self.colorSub = rospy.Subscriber('/camera/color/image_raw', Image, self._color_callback)
        self.colorPub = rospy.Publisher("camera/color/compressed", CompressedImage, queue_size=1)
        self.depthPub = rospy.Publisher("camera/depth/compressed", CompressedImage, queue_size=1)
        self.colorComp = None
        self.depthComp = None

    def compress_color_image(self,image):
        colorCV = self.cvbr.imgmsg_to_cv2(image,"bgr8")
        compressed = self.cvbr.cv2_to_compressed_imgmsg(colorCV,dst_format="jpg")
        compressed.header = image.header
        return compressed

    def compress_depth_image(self,image):
        image.encoding = "mono16"
        depthCV = self.cvbr.imgmsg_to_cv2(image,"mono16")
        compressed = self.cvbr.cv2_to_compressed_imgmsg(depthCV,dst_format="png")
        compressed.header = image.header
        compressed.format = '16UC1; compressedDepth'
        compressed.data = bytearray(b"\x00\x00\x00\x00\x88\x9c\x5c\xaa\x00\x40\x4b\xb7")+compressed.data
        return compressed

    def _color_callback(self,image):
        try:
            self.colorComp = self.compress_color_image(image)
        except CvBridgeError as e:
            print(e)

    def _depth_callback(self,image):
        try:
            self.depthComp = self.compress_depth_image(image)
        except CvBridgeError as e:
            print(e)

    def compress(self):
        if self.colorComp is not None:
            self.colorPub.publish(self.colorComp)
            self.colorComp = None
        if self.depthComp is not None:
            self.depthPub.publish(self.depthComp)
            self.depthComp = None

if __name__ == '__main__':
    rospy.init_node("image_compression", anonymous=True)
    ic = RSImageCompressor()
    while not rospy.is_shutdown():
        ic.compress()
