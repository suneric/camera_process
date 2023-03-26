#!/usr/bin/env python
import rospy
import numpy as np
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import math
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2
import argparse
from ids_detection.msg import DetectionInfo

class ACIMX219:
    def __init__(self, compressed = False):
        self.bridge=CvBridge()
        if compressed:
            self.color_sub = rospy.Subscriber('/arducam/image/compressed', CompressedImage, self._color_callback)
        else:
            self.color_sub = rospy.Subscriber('/arducam/image', Image, self._color_callback)
        self.info_sub = rospy.Subscriber('/arducam/cam_info', CameraInfo, self._caminfo_callback)
        # data
        self.cv_color = []
        self.width = 640
        self.height = 480
        self.cameraInfoUpdate = False

    def convertCompressedColorToCV2(self, colorComp):
        rawData = np.frombuffer(colorComp.data, np.uint8)
        return cv.imdecode(rawData, cv.IMREAD_COLOR)

    def ready(self):
        return self.cameraInfoUpdate and len(self.cv_color) > 0

    def image_size(self):
        return self.height, self.width

    def color_image(self):
        return self.cv_color

    def _caminfo_callback(self,data):
        if not self.cameraInfoUpdate:
            self.cameraInfoUpdate = True
            self.width = data.width
            self.height = data.height

    def _color_callback(self, data):
        if self.cameraInfoUpdate:
            try:
                if data._type == 'sensor_msgs/CompressedImage':
                    self.cv_color = self.convertCompressedColorToCV2(data)
                else:
                    self.cv_color = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

# realsense d435
class RSD435:
    # create a image view with a frame size for the ROI
    def __init__(self, compressed = False):
        print("create realsense d435 instance...")
        self.bridge=CvBridge()
        # camera information
        self.cameraInfoUpdate = False
        self.intrinsic = None
        # ros-realsense
        self.caminfo_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo, self._caminfo_callback)

        if compressed:
            self.depth_sub = rospy.Subscriber('/camera/depth/compressed', CompressedImage, self._depth_callback)
            self.color_sub = rospy.Subscriber('/camera/color/compressed', CompressedImage, self._color_callback)
        else:
            self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self._depth_callback)
            self.color_sub = rospy.Subscriber('/camera/color/image_raw', Image, self._color_callback)

        self.detection_sub = rospy.Subscriber("detection", DetectionInfo, self.detect_cb)

        # data
        self.cv_color = None
        self.cv_depth = None
        self.width = 640
        self.height = 480
        self.detectInfo = None

    def ready(self):
        return self.cameraInfoUpdate and self.cv_color is not None

    def image_size(self):
        return self.height, self.width

    def detect_cb(self,data):
        self.detectInfo = data

    #### data
    def depth_image(self):
        return self.cv_depth
    def color_image(self):
        return self.cv_color

    def _caminfo_callback(self, data):
        if self.cameraInfoUpdate == False:
            self.intrinsic = data.K
            self.width = data.width
            self.height = data.height
            self.cameraInfoUpdate = True

    def convertCompressedDepthToCV2(self, depthComp):
        fmt, type = depthComp.format.split(';')
        fmt = fmt.strip() # remove white space
        type = type.strip() # remove white space
        if type != "compressedDepth":
            raise Exception("Compression type is not 'compressedDepth'.")
        depthRaw = cv.imdecode(np.frombuffer(depthComp.data[12:], np.uint8),-1)
        if depthRaw is None:
            raise Exception("Could not decode compressed depth image.")
        return depthRaw

    def convertCompressedColorToCV2(self, colorComp):
        rawData = np.frombuffer(colorComp.data, np.uint8)
        return cv.imdecode(rawData, cv.IMREAD_COLOR)

    def _depth_callback(self, data):
        if self.cameraInfoUpdate:
            try:
                if data._type == 'sensor_msgs/CompressedImage':
                    self.cv_depth = self.convertCompressedDepthToCV2(data)
                else:
                    self.cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding) #"16UC1"
            except CvBridgeError as e:
                print(e)

    def _color_callback(self, data):
        if self.cameraInfoUpdate:
            try:
                if data._type == 'sensor_msgs/CompressedImage':
                    self.cv_color = self.convertCompressedColorToCV2(data)
                else:
                    self.cv_color = self.bridge.imgmsg_to_cv2(data, "bgr8")

                info = self.detectInfo
                names = ["door","door handle","human body","electric outlet","socket type B"]
                if info != None:
                    label = names[int(info.type)]
                    l,t,r,b=int(info.l),int(info.t),int(info.r),int(info.b)
                    cv.rectangle(self.cv_color,(l,t),(r,b),(0,255,0),2)
                    cv.putText(self.cv_color,label,(l-10,t-10),cv.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)

            except CvBridgeError as e:
                print(e)

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera', type=str, default="ac", help="either ac for arducam or rs for realsense")
    return parser.parse_args()

if __name__ == "__main__":
    args = get_args()
    rospy.init_node("rs_info", anonymous=True, log_level=rospy.INFO)

    cam = None
    if args.camera == "ac":
        cam = ACIMX219()
    elif args.camera == "acc":
        cam = ACIMX219(compressed = True)
    elif args.camera == "rs":
        cam = RSD435()
    elif args.camera == "rsc":
        cam = RSD435(compressed = True)
    else:
        print("no camera selected")

    rospy.sleep(1)
    rate = rospy.Rate(30)
    try:
        while not rospy.is_shutdown():
            if cam.ready():
                img = cam.color_image()
                #img = cam.depth_image()
                # print(img)
                cv.imshow('camera',img)
                cv.waitKey(1)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    cv.destroyAllWindows()
