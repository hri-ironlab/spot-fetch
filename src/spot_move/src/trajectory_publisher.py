#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Quaternion, TwistWithCovarianceStamped, Twist, Pose, PoseStamped
import sys
import math

class SpotMove():
    def __init__(self):
        self.trajectory_pub = rospy.Publisher('/spot/go_to_pose', PoseStamped, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher('/spot/cmd_vel', Twist, queue_size=10)

    def publish_trajectory(self, frame_id, x, y, z, yaw):
        waypoint = PoseStamped()
        waypoint.header.frame_id = str(frame_id)
        waypoint.pose.position.x = float(x)
        waypoint.pose.position.y = float(y)
        waypoint.pose.position.z = float(z)
        waypoint.pose.orientation = self.yaw_to_quaternion(float(yaw))
        
        self.trajectory_pub.publish(waypoint)
    
    def publish_cmd_vel(self, x, y, z):
        cmd_vel = Twist()
        cmd_vel.linear.x = float(x)
        cmd_vel.linear.y = float(y)
        cmd_vel.angular.z = float(z)

        self.cmd_vel_pub.publish(cmd_vel)
    
    def yaw_to_quaternion(self, yaw):
        quaternion = Quaternion()
        quaternion.w = math.cos(yaw / 2.0)
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        return quaternion

if __name__ == '__main__':
    rospy.init_node('spot_move')
    spot_move = SpotMove()
    rospy.sleep(2)
   
    if len(sys.argv) == 4:
        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            spot_move.publish_cmd_vel(sys.argv[1], sys.argv[2], sys.argv[3])
            rate.sleep()
    elif len(sys.argv) == 6:
        spot_move.publish_trajectory(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5])
    else:
        print("usage: \nrosrun spot_move trajectory_publisher frame_id position.x position.y position.z yaw \nor\nrosrun spot_move trajectory_publisher linear.x linear.y angular.z")

