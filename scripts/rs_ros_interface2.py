#!/usr/bin/env python

"""
republish image with compressed
reference:
https://gist.github.com/awesomebytes/958a5ef9e63821a28dc05775840c34d9
https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/align-depth2color.py
"""
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import numpy as np
import pyrealsense2 as rs

def publish_images(align, cvBridge, colorPub, depthPub, colorCompPub, depthCompPub):
    depthFrame = align.get_depth_frame()
    colorFrame = align.get_color_frame()
    if not depthFrame or not colorFrame:
        return

    try:
        colorArr = np.asanyarray(colorFrame.get_data())
        depthArr = np.asanyarray(depthFrame.get_data())
        colorImg = cvBridge.cv2_to_imgmsg(colorArr, encoding='bgr8')
        depthImg = cvBridge.cv2_to_imgmsg(depthArr, encoding='mono16')
        colorCompImg = cvBridge.cv2_to_compressed_imgmsg(colorArr, dst_format='jpg')
        depthCompImg = cvBridge.cv2_to_compressed_imgmsg(depthArr, dst_format='png')
        depthCompImg.format='16UC1; compressedDepth'
        depthCompImg.data = "\x00\x00\x00\x00\x88\x9c\x5c\xaa\x00\x40\x4b\xb7"+depthCompImg.data
        stamp = rospy.Time.now()
        colorImg.header.stamp = stamp
        depthImg.header.stamp = stamp
        colorCompImg.header.stamp = stamp
        depthCompImg.header.stamp = stamp
        colorPub.publish(colorImg)
        depthPub.publish(depthImg)
        colorCompPub.publish(colorCompImg)
        depthCompPub.publish(depthCompImg)
    except CvBridgeError as e:
        print(e)

def stream(profile, width=640, height=480):
    infoPub = rospy.Publisher("camera/color/camera_info", CameraInfo, queue_size=1)
    colorPub = rospy.Publisher("camera/color/image_raw", Image, queue_size=1)
    depthPub = rospy.Publisher("camera/depth/image_rect_raw", Image, queue_size=1)
    colorCompPub = rospy.Publisher("camera/color/compressed", CompressedImage, queue_size=1)
    depthCompPub = rospy.Publisher("camera/depth/compressed", CompressedImage, queue_size=1)
    cvBridge = CvBridge()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is:", depth_scale)
    align = rs.align(rs.stream.color) # align depth frame to color frame
    color = profile.get_stream(rs.stream.color)
    intr = color.as_video_stream_profile().get_intrinsics()
    try:
        while not rospy.is_shutdown():
            info = CameraInfo()
            info.width = intr.width
            info.height = intr.height
            info.D = intr.coeffs
            info.K = [intr.fx,0,intr.ppx,0,intr.fy,intr.ppy,0,0,1.0]
            info.P = [intr.fx,0,intr.ppx,0,0,intr.fy,intr.ppy,0,0,0,1.0,0]
            info.R = [1.0,0,0,0,1.0,0,0,0,1.0]
            infoPub.publish(info)
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            publish_images(aligned_frames,cvBridge,colorPub,depthPub,colorCompPub,depthCompPub)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    rospy.init_node("realsese_d435", anonymous=True)
    width,height,frame_rate = 640,480,30
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, frame_rate)
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, frame_rate)
    profile = pipeline.start(config)
    if len(profile.get_device().sensors) == 0:
        print("Unable to open camera.")
    else:
        stream(profile, width, height)
    pipeline.stop()
