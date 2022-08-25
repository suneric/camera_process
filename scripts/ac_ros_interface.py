#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
import numpy as np
import os

def focusing(val):
	value = (val << 4) & 0x3ff0
	data1 = (value >> 8) & 0x3f
	data2 = value & 0xf0
	os.system("i2cset -y 6 0x0c %d %d" % (data1,data2))

def sobel(img):
	img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
	img_sobel = cv2.Sobel(img_gray,cv2.CV_16U,1,1)
	return cv2.mean(img_sobel)[0]

def laplacian(img):
	img_gray = cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
	img_sobel = cv2.Laplacian(img_gray,cv2.CV_16U)
	return cv2.mean(img_sobel)[0]

def publish_image(image,imgPub,infoPub,compressPub,cvbr,dim):
	tw,th = dim[0],dim[1]
	h,w,_ = image.shape
	wscale = tw/w
	hscale = th/h
	s = wscale
	if hscale < wscale:
		s = hscale
	dsize = (int(s*w),int(s*h))
	img = cv2.resize(image,dsize) # resize
	cx,cy = img.shape[1]/2,img.shape[0]/2
	img = img[int(cy-th/2):int(cy+th/2),int(cx-tw/2):int(cx+tw/2)] # crop image
	imgPub.publish(cvbr.cv2_to_imgmsg(img,"bgr8"))
	compressPub.publish(cvbr.cv2_to_compressed_imgmsg(img))
	info = CameraInfo()
	info.width = dim[0]
	info.height = dim[1]
	infoPub.publish(info)


# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Defaults to 1280x720 @ 60fps
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of the window on the screen
def gstreamer_pipeline (capture_width=1280, capture_height=720, display_width=1280, display_height=720, framerate=60, flip_method=0):
    return ('nvarguscamerasrc ! '
    'video/x-raw(memory:NVMM), '
    'width=(int)%d, height=(int)%d, '
    'format=(string)NV12, framerate=(fraction)%d/1 ! '
    'nvvidconv flip-method=%d ! '
    'video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! '
    'videoconvert ! '
    'video/x-raw, format=(string)BGR ! appsink'  % (capture_width,capture_height,framerate,flip_method,display_width,display_height))

def stream(cap,dim,fps):
	imgPub = rospy.Publisher("arducam/image", Image, queue_size=1)
    infoPub = rospy.Publisher("arducam/cam_info", CameraInfo, queue_size=1)
	compressPub = rospy.Publisher('arducam/image/compressed', CompressedImage, queue_size=1)
    cvbr = CvBridge()

    max_index = 10
    max_value = 0.0
    last_value = 0.0
    dec_count = 0
    focal_distance = 10
    focus_finished = False
    rate = rospy.Rate(fps)
    while not rospy.is_shutdown():
        ret_val, img = cap.read()
		publish_image(img,imgPub,infoPub,compressPub,cvbr,dim)
        if dec_count < 6 and focal_distance < 1000:
            focusing(focal_distance) #Adjust focus
            val = laplacian(img) #Take image and calculate image clarity
            if val > max_value: #Find the maximum image clarity
                max_index = focal_distance
                max_value = val
            if val < last_value: #If the image clarity starts to decrease
                dec_count += 1
            else:
                dec_count = 0
            if dec_count < 6: #Image clarity is reduced by six consecutive frames
                last_value = val
                focal_distance += 10 #Increase the focal distance
        elif not focus_finished:
            focusing(max_index) #Adjust focus to the best
            focus_finished = True

        rate.sleep()

if __name__ == '__main__':
	rospy.init_node("arducam_imx219", anonymous=True)
    pipe = gstreamer_pipeline()
    cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
		print("Unable to open camera")
    else:
		stream(cap,dim=(256,256),fps=30)

    cap.release()
