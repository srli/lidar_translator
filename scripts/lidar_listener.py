#!/usr/bin/env python
import sys
import math
import rospy

import roslib; roslib.load_manifest('lidar_translator')

from lidar_translator.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64

def callback(data):
	print data.closest_obj

def listener():
    rospy.init_node('lidar_listener_node', anonymous=True)
    rospy.Subscriber('lidar_output', lidar_output, callback)

    pub = rospy.Publisher("clean_lidar", Float64)
    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        pass
        #idle_gesture(last_message_received_time, pub, r)


if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException: pass