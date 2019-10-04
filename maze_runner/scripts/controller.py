#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import time

# dist_closest = 0.0
# angle_closest = 0.0
# dist_target = 0.3
# angle_parallel = 1.57079632679 # pi / 2

# def update_data(msg):
#     global dist_closest, angle_closest
#     index_closest, dist_closest = min(enumerate(msg.ranges), key=lambda x: x[1])
#     angle_closest = msg.angle_min + index_closest * msg.angle_increment

# rospy.init_node('controller')
# scan_sub = rospy.Subscriber('scan', LaserScan, update_data)
# cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# rate = rospy.Rate(10)

# while not rospy.is_shutdown():
#     dist_error = dist_closest - dist_target
#     curr_angle = angle_parallel - angle_closest # with respect to parallel
#     target_angle = math.asin(0.3 * dist_error) # with respect to parallel
#     angle_error = curr_angle - target_angle

#     vel = Twist()
#     vel.linear.x = 0.2
#     vel.angular.z = -0.3 * angle_error

#     cmd_vel_pub.publish(vel)
#     rate.sleep()
    



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

    if dist_ahead < dist_ahead_threshold:
        time_last_saw_wall = time_now
        vel.angular.z = 1.5
    else:
        vel.linear.x = 0.2
        if dist_diag > dist_diag_threshold:
            vel.angular.z = -1.0

    if time_now - time_last_saw_wall > 20:
        vel.angular.z = 0.0

    cmd_vel_pub.publish(vel)
    rate.sleep()
