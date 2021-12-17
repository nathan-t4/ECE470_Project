#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Point

block_height = 0.05
hokuyo_x = -0.025
hokuyo_y = 0.025
object_world = Point()

def hokuyo2world(range_min, range_max, angle_min, angle_max, angle_increment, ranges, block, order):

	global object_world, block_height, hokuyo_x, hokuyo_y

	object_range = []
	object_angle = []

	print ranges
	print angle_min
	print angle_increment
	print np.size(ranges)
	#cabinet_angle = np.array([-0.387, -0.1740])
	ur3_angle = [2.084, 2.2689]
	k = 0
	for i in range(0, np.size(ranges)): 
		cur_angle = angle_min + angle_increment*i
		#is_cabinet = cabinet_angle[0] < cur_angle < cabinet_angle[1]
		is_ur3 = ur3_angle[0] < cur_angle < ur3_angle[1]
		if (ranges[i] < 0.45 and not is_ur3):
			object_range.append(ranges[i])
			object_angle.append(cur_angle)
			k = k + 1

	block_range = []
	prev_angle = 0
	prev_distance = object_range[0]
	j = 0
	limit = 15
	if block+(order-1) == 3:
	   limit = 20

	for i in range(0, np.size(object_range)):
		delta_angle = object_angle[i] - prev_angle
		delta_distance = object_range[i] - prev_distance
		if delta_distance > 0.1 or j > 0:
		   j = j + 1
		   if j < limit: #15
		      continue
		
		if not delta_angle == angle_increment:
		   block_range.append(i)
		prev_distance = object_range[i]
		prev_angle = object_angle[i]
	
	#print(ranges)
	#print(block_range)
	#print(object_range)
	#print(object_angle)
	object_x = []
	object_y = []

	chosen_block_range = [block_range[block-1], block_range[block]];

	for i in range(chosen_block_range[0], chosen_block_range[1]):
		d = object_range[i]
		theta = object_angle[i]
		object_x.append(d*np.cos(theta) + hokuyo_x)
		object_y.append(d*np.sin(theta) + hokuyo_y)

	#object_height = object_y[np.size(object_range)-1] - object_y[0]
	#print(object_height)

	avg_x = np.average(object_x)
	avg_y = np.average(object_y)
	object_world.x = avg_x + block_height
	object_world.y = avg_y
	object_world.z = block_height
	#object_world.x = avg_x
	#object_world.y = avg_y
	#object_world.z = block_height/2
	#object_world.x = avg_x + 0.01/2 # x dimension of the object divided by 2
	#object_world.y = avg_y
	#object_world.z = 0.08/2 # z dimension of the object divided by 2
	#print(object_world.x)
	#print(object_world.y)
	#print(object_world.z)
	#object_world.x = 0.2
	#object_world.y = 0.1
	#object_world.z = 0.05

	return object_world
	



