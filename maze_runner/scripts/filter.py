#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

dist_ahead = 0.0
dist_diag = 0.0
dist_ahead_threshold = 0.2
dist_diag_threshold = 0.3
Kp = 0.03
Kd = 0.1
Ki = 0.1
derivative = 0 #might include later
integral = 0

def update_dist_ahead(msg):
    global dist_ahead 
    dist_ahead = msg.data

def update_dist_diag(msg):
    global dist_diag, integral
    dist_diag = msg.ranges[int(0.25 / msg.angle_increment)]
    integral = integral + dist_diag

def pid():
    global dist_diag_threshold, dist_diag
    error = abs(dist_diag_threshold - dist_diag)
    global Kp, Kd, integral, Ki
    P_value = Kp * error
    # D_value = Kd * (error - Derivator)
    # Derivator = error
    integral = integral + error
    I_value = integral * Ki
    return P_value


rospy.init_node('filter')
dist_sub = rospy.Subscriber('distance', Float32, update_dist_ahead)
scan_sub = rospy.Subscriber('scan', LaserScan, update_dist_diag)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(10) #can change this later

while not rospy.is_shutdown():
    vel = Twist()
    if dist_ahead < dist_ahead_threshold:
        vel.angular.z = 1.0
        vel.linear.x = pid()
    else:
        vel.linear.x = 0.1
        if dist_diag > dist_diag_threshold:
            vel.angular.z = -0.5

    print("dist_ahead = ", dist_ahead)
    print("dist_diag = ", dist_diag)
    cmd_vel_pub.publish(vel)
    rate.sleep()
