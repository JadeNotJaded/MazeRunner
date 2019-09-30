#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

dist_ahead = 0.0
dist_diag = 0.0
dist_ahead_threshold = 0.15
dist_diag_threshold = 0.4

def update_dist_ahead(msg):
    global dist_ahead 
    dist_ahead = msg.data

def update_dist_diag(msg):
    global dist_diag 
    dist_diag = msg.ranges[int(0.5 / msg.angle_increment)]

rospy.init_node('filter')
dist_sub = rospy.Subscriber('distance', Float32, update_dist_ahead)
scan_sub = rospy.Subscriber('scan', LaserScan, update_dist_diag)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    vel = Twist()
    if dist_ahead < dist_ahead_threshold:
        vel.angular.z = 1.0
    else:
        vel.linear.x = 0.2
        if dist_diag > dist_diag_threshold:
            vel.angular.z = -1.0

    print("dist_ahead = ", dist_ahead)
    print("dist_diag = ", dist_diag)
    cmd_vel_pub.publish(vel)
    rate.sleep()
