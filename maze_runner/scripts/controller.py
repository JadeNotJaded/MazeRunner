#!/usr/bin/env python
# authors: Jade Garisch and Addison Phitha

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

dist_ahead = 0.0
dist_diag = 0.0
dist_ahead_threshold = 0.4
dist_diag_threshold = 0.4
time_last_saw_wall = time.time()
dist_closest = 0
angle_closest = 0

def update_dist_ahead(msg):
    global dist_ahead 
    dist_ahead = msg.data

def update_dist_diag(msg):
    global dist_diag 
    dist_diag = msg.ranges[int(1.57 / msg.angle_increment)]

rospy.init_node('controller')
dist_sub = rospy.Subscriber('distance', Float32, update_dist_ahead)
scan_sub = rospy.Subscriber('scan', LaserScan, update_dist_diag)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    vel = Twist()
    time_now = time.time()

    #if about to crash into a wall don't move forward and turn left 
    if dist_ahead < dist_ahead_threshold:
        time_last_saw_wall = time_now
        vel.angular.z = 1.5
    else:
        #else move forward
        vel.linear.x = 0.2
        # if the diagonal distance to the front right is too small move to the right
        if dist_diag > dist_diag_threshold:
            vel.angular.z = -1.0

    #if the time that we weren't close to the wall is greater than 20 seconds find a wall
    if time_now - time_last_saw_wall > 20:
        vel.angular.z = 0.0

    cmd_vel_pub.publish(vel)
    rate.sleep()
