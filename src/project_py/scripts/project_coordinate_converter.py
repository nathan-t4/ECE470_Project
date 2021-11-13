#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point

block_height = 0.08
hokuyo_x = -0.025
hokuyo_y = 0.025
object_world = Point()

def hokuyo2world(range_min, range_max, angle_min, angle_max, angle_increment, ranges):

	global object_world, block_height, hokuyo_x, hokuyo_y
	object_range = []
	object_angle = []
	k = 0
	for i in range(0, np.size(ranges)): 
		if (ranges[i] < range_max and np.size(ranges)-i > 30):
			object_range.append(ranges[i])
			object_angle.append(angle_min + angle_increment*i)
			k = k + 1

	object_x = []
	object_y = []
	for i in range(0, np.size(object_range)):
		d = object_range[i]
		theta = object_angle[i]
		object_x.append(d*np.cos(theta) + hokuyo_x)
		object_y.append(d*np.sin(theta) + hokuyo_y)

	avg_x = np.average(object_x)
	avg_y = np.average(object_y)
	object_world.x = avg_x + block_height/2
	object_world.y = avg_y
	object_world.z = block_height

	return object_world
	



