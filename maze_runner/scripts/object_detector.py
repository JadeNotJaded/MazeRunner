#! /usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

def bearing(angle_min, angle_inc, i):
    return angle_min + i * angle_inc

def process(msg):
    # below we convert the coordinates from polar to cartesian coordinates
    min = msg.angle_min
    inc = msg.angle_increment
    locations = []
    for i in range(len(msg.ranges)):
        angle = bearing(min, inc, i)
        r = msg.ranges[i]
        locations.append((r*math.cos(angle), r*math.sin(angle)))
    # find the object that is closest ahead in our defined rectangular area
    closest_ahead = 10.0
    for x, y in locations:
        if (y > -0.15) and (y < 0.15) and (x > 0):
            if x < closest_ahead:
                closest_ahead = x

    pub.publish(closest_ahead)



rospy.init_node('obstacle_detector')
sub = rospy.Subscriber('scan', LaserScan, process)
pub = rospy.Publisher('distance', Float32, queue_size=1)
rospy.spin()
