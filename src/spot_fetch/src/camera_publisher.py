#!/usr/bin/env python3
# This can be used for debugging vision if you don't have a ZED with you
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

bridge = CvBridge()
cap = cv2.VideoCapture(0)
pub = rospy.Publisher("/zed2i/zed_node/left/image_rect_color", Image)

while not rospy.is_shutdown():
    ret,frame = cap.read()
    pub.publish(bridge.cv2_to_imgmsg(frame))