#!/usr/bin/env python
import sys
import math
import rospy
from std_msgs.msg import String, Float64
from socket import *

class LidarUDP:

	def __init__(self):
		self.sock = socket(AF_INET, SOCK_DGRAM)
		self.sock.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
		self.sock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)

		rospy.Subscriber("closest_obj", Float64, self.callback)
		rospy.Subscriber("lidar_theta", Float64, self.callback2)

	def callback(self, data):
		UDP_IP = "255.255.255.255" # universal broadcasting
		UDP_PORT = 60012

		MESSAGE = str(data.data)
		print MESSAGE
		self.sock.sendto(MESSAGE, (UDP_IP, UDP_PORT)) #publishes to udp connection
		print "closest_obj sent"

	def callback2(self, data):
		UDP_IP = "255.255.255.255" # universal broadcasting
		UDP_PORT = 60050

		MESSAGE = str(data.data)
		print MESSAGE
		self.sock.sendto(MESSAGE, (UDP_IP, UDP_PORT)) #publishes to udp connection
		print "lidar_theta sent"

def main(args):
	'''Initializes and cleanup ros node'''
	rospy.init_node('LidarUDP', anonymous=True)
	lt = LidarUDP()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print "Shutting down ROS LidarTranslator"

if __name__ == '__main__':
	main(sys.argv)