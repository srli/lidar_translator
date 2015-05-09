#!/usr/bin/env python
import sys
import math
import rospy

import roslib; roslib.load_manifest('lidar_translator')

from lidar_translator.msg import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float64
from socket import *


class LidarTranslator:

	def __init__(self):
		# debug constants
		self.rendered = False
		self.publish_interval = 50
		self.current_interval = 0

		# Lidar geometric constants
		self.angle_min = -1*(math.pi/2)
		self.angle_max =  1*(math.pi/2)
		self.inch_per_meter = 39.37
		self.dist_max = 5*self.inch_per_meter #0.85 inch width per beam at 5 metres
		self.incr_max = 0.5*self.inch_per_meter #difference defines new objects
		self.robot_width = 12

		# ROS initialization
		# self.pub = rospy.Publisher('closest_obj', Float64)
		# self.pub2 = rospy.Publisher('lidar_status', String)
		# self.pub3 = rospy.Publisher('lidar_theta', Float64)
		self.pub = rospy.Publisher('lidar_output', lidar_output)

		rospy.Subscriber("scan", LaserScan, self.callback)

	def callback(self, data):
		# print data.ranges[500]
		# print len(data.ranges)
		data.ranges = [self.inch_per_meter * r for r in data.ranges]
		data.ranges.reverse()

		# breaks terribly if self.angle_min and _max are impossible
		min_index = int(math.ceil(
			(self.angle_min-data.angle_min)/data.angle_increment))
		max_index = int(math.floor(
			(self.angle_max-data.angle_min)/data.angle_increment))
		filtered_ranges = data.ranges[min_index:max_index+1]

		closest_obj = min(filtered_ranges) * self.inch_per_meter

		index_ranges = self.obj_angles(data, min_index, max_index)
		obstacles = self.ranges_to_obstacles(data, index_ranges)
		min_dist, min_obs = self.closest_forward_obstacle(obstacles)
		self.pub.publish(min_dist)

		theta = self.forward_wall_angle(obstacles)
		self.pub3.publish(theta)

		# Publishing is slowed down to ease human readability
		if self.current_interval < self.publish_interval:
			self.current_interval += 1
		else:
			self.current_interval = 0
			self.pub2.publish(str(obstacles))

		if self.rendered == False:
			self.rendered = True
			print min_index
			print max_index
			print data.ranges[620:651]
			print index_ranges
			print obstacles
			print min_dist, min_obs

	def obj_angles(self, data, min_index, max_index):
		index_ranges = []
		start_index, end_index = None, None
		for i in range(min_index, max_index+1):
			if data.ranges[i] < self.dist_max:
				if start_index == None:
					start_index = i
					end_index = i
				else:
					if abs(data.ranges[end_index]-data.ranges[i]) < self.incr_max:
						end_index = i
					else:
						index_ranges.append((start_index, end_index))
						start_index, end_index = i, i
			else:
				if start_index == None or end_index == None:
					continue
				else:
					index_ranges.append((start_index, end_index))
					start_index, end_index = None, None
		if start_index == None or end_index == None:
			pass
		else:
			index_ranges.append((start_index, end_index))

		return index_ranges

	def ranges_to_obstacles(self, data, index_ranges):
		"""Translates each index_range to x,y coordinates
		and classifies them as cart, human, or wall"""
		obstacles = []
		for index_range in index_ranges:
			start_index = index_range[0]
			end_index = index_range[1]
			mid_index = start_index+(end_index-start_index)/2

			left_angle = data.angle_min + start_index*data.angle_increment
			left_x = data.ranges[start_index] * math.sin(left_angle)
			left_y = data.ranges[start_index] * math.cos(left_angle)

			right_angle = data.angle_min + end_index*data.angle_increment
			right_x = data.ranges[end_index] * math.sin(right_angle)
			right_y = data.ranges[end_index] * math.cos(right_angle)

			range_min = min(data.ranges[start_index:end_index+1])
			mid_angle = data.angle_min + mid_index*data.angle_increment
			width_angle = data.angle_increment * (end_index-start_index)
			x = range_min*math.sin(mid_angle)
			y = range_min*math.cos(mid_angle)
			radius = 2*range_min*math.sin(width_angle/2)

			if radius == 5:
				obj_type = "easter bunny"
			if radius < 2:
				obj_type = "cart"
			elif radius < 30:
				obj_type = "human"
			else:
				obj_type = "wall"


			radius = round(radius, 2)
			x = round(x, 2)
			y = round(y, 2)
			xs = [left_x, right_x]
			ys = [left_y, right_y]

			if obj_type == "wall":
				radius = 1 #1 inch buffer
				obstacles.append(Obstacle(xs, ys, radius, obj_type))
			else:
				obstacles.append(Obstacle([x], [y], radius, obj_type))

		return obstacles

	# overestimates distances of under 6 inches
	def closest_forward_obstacle(self, obstacles):
		"""returns distance and closest object within robot's forward path

		Args:
			obstacles: list of Obstacle objects
		"""
		min_obs = None
		min_dist = 1000
		for obs in obstacles:
			if (min(obs.xs)-obs.radius < self.robot_width/2 and
				max(obs.xs)+obs.radius > -self.robot_width/2):
				obs_dist = min(obs.ys)
				if obs_dist < min_dist:
					min_dist = obs_dist #not good assumption for walls
					min_obs = obs
		return min_dist, min_obs


	def forward_wall_angle(self, obstacles):
		"""returns the difference in angle between the lidar
		and the wall in front of it, if it exists.
		0 == perpendicular,
		positive for left, negative for right

		Args:
			obstacles: list of Obstacle objects
		"""
		angle = 0 #answer in case of error
		for obs in obstacles:
			if obs.obs_type == "wall":
				if (min(obs.xs)-obs.radius < self.robot_width/2 and
					max(obs.xs)+obs.radius > -self.robot_width/2):
					dx = obs.xs[1] - obs.xs[0] #right - left
					dy = obs.ys[1] - obs.ys[0]
					angle = math.atan(dy/dx)
		return angle

class Obstacle(object):
	"""represents any obstacle seen by lidar

	Typically created by the LidarTranslator,
	and kept inside a list of Obstacles"""
	def __init__(self, xs, ys, radius, obs_type):
		self.xs = xs
		self.ys = ys
		self.radius = radius
		self.obs_type = obs_type

	def add_leg(self, x, y, radius):
		"""used for procedurally generating multileg Obstacles
		like humans or carts"""
		self.xs.append(x)
		self.ys.append(y)
		self.radius = max(self.radius, radius)

	def __repr__(self):
		"""called whenever Obstacle is printed as member of list"""
		return self.__str__()

	def __str__(self):
		return "{%s %s %s %s}" % (self.xs, self.ys, self.radius, self.obs_type)





def main(args):
    '''Initializes and cleanup ros node'''
    rospy.init_node('LidarTranslator', anonymous=True)
    lt = LidarTranslator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS LidarTranslator"

if __name__ == '__main__':
    main(sys.argv)