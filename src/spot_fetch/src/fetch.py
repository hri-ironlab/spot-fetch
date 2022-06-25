#!/usr/bin/env python3
# A program to get the Spot robot from Boston Dynamics to go fetch a tennis ball
import rospy
import math
from sensor_msgs.msg import Image, CameraInfo, Imu
import numpy as np
import tennisball # best module name ever :)
import cv2
import tf2_ros
import tf2_geometry_msgs
from image_geometry import PinholeCameraModel as PCM
from geometry_msgs.msg import PoseStamped, Quaternion
from std_srvs.srv import Trigger
from dynamic_reconfigure.server import Server
from spot_fetch.cfg import TennisBallConfig
from cv_bridge import CvBridge

cvb = CvBridge()
pcm = PCM()
pipeline = tennisball.TennisBallPipeline()
latest_depth = None # stores the latest depth data
trajectory_pub = None # publisher for the /spot/go_to_pose topic
latest_point = None # stores the latest 3D ball detection location
latest_orientation = None # stores the latest ZED IMU orientation (currently unused)
tfBuffer = None # transform stuff
listener = None # transform stuff
display = False # set to True when you have a monitor & want to see debug image
# If `display` is set to True when no monitor is attached, the program will crash.

# Callback for new IMU data
def imuCallback(msg):
    global latest_orientation
    latest_orientation = msg.orientation

# Callback for when dynamic reconfigure data is changed (for the vision pipeline)
# For some reason, changing the dynamic reconfigure data isn't updating vision
# and I'm not sure why. TODO fix this
def confCallback(config, level):
    rospy.loginfo("Config changed")
    print(config["contours_max_ratio"])
    pipeline.__hsv_threshold_hue = [config["hue_min"], config["hue_max"]]
    pipeline.__hsv_threshold_saturation = [config["sat_min"], config["sat_max"]]
    pipeline.__hsv_threshold_value = [config["val_min"], config["val_max"]]
    pipeline.__filter_contours_min_area = config["contours_min_area"]
    pipeline.__filter_contours_min_perimeter = config["contours_min_perimeter"]
    pipeline.__filter_contours_min_width = config["contours_min_width"]
    pipeline.__filter_contours_max_width = config["contours_max_width"]
    pipeline.__filter_contours_min_height = config["contours_min_height"]
    pipeline.__filter_contours_max_height = config["contours_max_height"]
    pipeline.__filter_contours_solidity = [config["contours_min_solidity"], config["contours_max_solidity"]]
    pipeline.__filter_contours_max_vertices = config["contours_max_vertices"]
    pipeline.__filter_contours_min_vertices = config["contours_min_vertices"]
    pipeline.__filter_contours_min_ratio = config["contours_min_ratio"]
    pipeline.__filter_contours_max_ratio = config["contours_max_ratio"]
    return config

# Service handler for /fetch/fetch_tennis_ball
def go_fetch(req):
    if latest_point is None:
        rospy.loginfo("No ball found. Spot is sad :(")
    else:
        rospy.loginfo("Fetching")
        publish_trajectory("body", latest_point[0], latest_point[1], 0, 0)

# Callback for new image data
def callback(img):
    global latest_point
    cv_image = np.frombuffer(img.data, dtype=np.uint8).reshape(img.height, img.width, -1) # cv_bridge wasn't working for me, this is a workaround
    pipeline.process(cv_image) # tell the vision pipeline to process the image
    contours = pipeline.filter_contours_output # get the filtered contours/final output (see tennisball.py for more info)
    cv2.drawContours(cv_image, pipeline.find_contours_output, -1, (0, 255, 0), 2) # draw all found contours (not just the filtered ones) onto the debug image
    try:
        contour = max(contours, key=cv2.contourArea) # try to get the biggest contour
    except:
        # if finding the biggest contour fails, there are no contours
        latest_point = None
        rospy.loginfo("No ball found.")
        if display: cv2.imshow("", cv_image)
        if display: cv2.waitKey(1)
        return
    # yay, the function didn't return, the biggest contour exists!
    x,y,w,h = cv2.boundingRect(contour) # get its bounding box
    cv2.rectangle(cv_image, (x, y), (x + w, y + h), (66, 135, 245), 2) # draw it on the debug image
    if display: cv2.imshow("", cv_image)
    cx = x + (w // 2) # get the center of the contour
    cy = y + (h // 2)
    if display: cv2.waitKey(1)
    d = latest_depth[int(cy)][int(cx)] # get the depth at the center of the contour.
    # this may be slightly off because we are using the left camera for detection and depth is likely centered but I'm not sure
    pt = pcm.projectPixelTo3dRay((cx, cy)) * d # project into 3D space
    x,y,z = pt # get the x, y, and z values of that projection
    pt = (z, -x, -y) # the ZED has a different orientation than Spot, so this corrects for that
    latest_point = left_zed_to_body(pt) # transform the point to the body frame
    rospy.loginfo(latest_point) # print out the latest point

# Transforms a point from the left ZED camera to the middle of Spot
def left_zed_to_body(point):
    # create a PoseStamped to transform with the point's coordinates
    ps = PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "zed2i_left_camera_frame"
    ps.pose.position.x = point[0]
    ps.pose.position.y = point[1]
    ps.pose.position.z = point[2]
    ps.pose.orientation.x = 0
    ps.pose.orientation.y = 0
    ps.pose.orientation.z = 0
    ps.pose.orientation.w = 1

    try:
        transform = tfBuffer.lookup_transform("zed2i_base_link", ps.header.frame_id, ps.header.stamp, rospy.Duration(0.25))
        # I previously transformed using the ZED IMU orientation also, but that caused problems when the robot spun around.
        # We decided to just assume the ZED is pointing straight forward.
        pst = tf2_geometry_msgs.do_transform_pose(ps, transform) # pst = PoseStamped transformed
        pst.pose.position.x += 0.35 # the ZED is about 0.35 meters from the center of Spot
        return (pst.pose.position.x, pst.pose.position.y, pst.pose.position.z) # Return the transformed point
    except Exception as e:
        # If the transform fails, log the error and return None
        rospy.logerr(e)
        return None

# Callback to store the latest depth data
def depthCallback(img):
    global latest_depth
    cv_image = np.frombuffer(img.data, dtype=np.float32).reshape(img.height, img.width, -1)
    latest_depth = cv_image

# Callback to store the latest camera info (for 2D -> 3D)
def infoCallback(info):
    pcm.fromCameraInfo(info)

def yaw_to_quaternion(yaw):
    quaternion = Quaternion()
    quaternion.w = math.cos(yaw / 2.0)
    quaternion.x = 0.0
    quaternion.y = 0.0
    quaternion.z = math.sin(yaw / 2.0)
    return quaternion

# Publishes a trajectory to Spot (tells Spot to go somewhere)
def publish_trajectory(frame_id, x, y, z, yaw):
    global trajectory_pub
    waypoint = PoseStamped()
    waypoint.header.frame_id = str(frame_id)
    waypoint.pose.position.x = float(x)
    waypoint.pose.position.y = float(y)
    waypoint.pose.position.z = float(z)
    waypoint.pose.orientation = yaw_to_quaternion(float(yaw))

    trajectory_pub.publish(waypoint)

# Initialization
def fetch():
    global trajectory_pub, tfBuffer, listener
    print("Initializing a friendly game of fetch...")
    rospy.init_node('fetch', anonymous=False)

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    trajectory_pub = rospy.Publisher('/spot/go_to_pose', PoseStamped, queue_size=10) # trajectory publisher
    rospy.Subscriber("/zed2i/zed_node/left/image_rect_color", Image, callback) # subscribers
    rospy.Subscriber("/zed2i/zed_node/depth/depth_registered", Image, depthCallback)
    rospy.Subscriber("/zed2i/zed_node/left/camera_info", CameraInfo, infoCallback)
    rospy.Subscriber("/zed2i/zed_node/imu/data", Imu, imuCallback)
    print("Initialized and subscribed, spinning")
    s = rospy.Service("/fetch/fetch_tennis_ball", Trigger, go_fetch) # fetch service server

    srv = Server(TennisBallConfig, confCallback) # dynamic reconfigure server

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    fetch()
