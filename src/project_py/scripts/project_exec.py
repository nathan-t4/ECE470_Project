#!/usr/bin/env python
import copy
import time
import rospy
import sys
import numpy as np
import argparse
import sys

from project_header import *
from project_func import *
from spawn_block import *
from sensor_msgs.msg import LaserScan
from project_coordinate_converter import *
from geometry_msgs.msg import Point

# 20Hz
SPIN_RATE = 20 

# UR3 home location
home = [0*PI/180.0, -90*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0, 0*PI/180.0]

# UR3 current position, using home position for initialization
current_position = copy.deepcopy(home)

thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

digital_in_0 = 0
analog_in_0 = 0.0

suction_on = True
suction_off = False

current_io_0 = False
current_position_set = False

ranges = []
angle_increment = 0
time_increment = 0
angle_min = 0
angle_max = 0
range_min = 0
range_max = 0

block_goal = Point()
block_goal.x = -0.6
block_goal.y = 0.1
block_goal.z = 0.1

def laser_callback(msg):
	global ranges, angle_increment, time_increment, angle_min, angle_max, range_min, range_max
	angle_increment = msg.angle_increment # [rad]
	time_increment = msg.time_increment # [sec]
	angle_min = msg.angle_min # [rad]
	angle_max = msg.angle_max # [rad]
	range_min = msg.range_min # [m]
	range_max = msg.range_max # [m]
	ranges = msg.ranges
"""
Whenever ur3/gripper_input publishes info this callback function is called.
"""
def input_callback(msg):

	global digital_in_0, analog_in_0
	digital_in_0 = msg.DIGIN
	digital_in_0 = digital_in_0 & 1 # Only look at least significant bit, meaning index 0
	analog_in_0 = msg.AIN0

"""
Whenever ur3/position publishes info, this callback function is called.
"""
def position_callback(msg):

	global thetas, current_position, current_position_set

	thetas[0] = msg.position[0]
	thetas[1] = msg.position[1]
	thetas[2] = msg.position[2]
	thetas[3] = msg.position[3]
	thetas[4] = msg.position[4]
	thetas[5] = msg.position[5]

	current_position[0] = thetas[0]
	current_position[1] = thetas[1]
	current_position[2] = thetas[2]
	current_position[3] = thetas[3]
	current_position[4] = thetas[4]
	current_position[5] = thetas[5]

	current_position_set = True

"""
Function to control the suction cup on/off
"""
def gripper(pub_cmd, loop_rate, io_0):

	global SPIN_RATE, thetas, current_io_0, current_position

	error = 0
	spin_count = 0
	at_goal = 0

	current_io_0 = io_0

	driver_msg = command()
	driver_msg.destination = current_position
	driver_msg.v = 1.0
	driver_msg.a = 1.0
	driver_msg.io_0 = io_0   #?????????????????
	pub_cmd.publish(driver_msg)

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			#rospy.loginfo("Goal is reached!")
			at_goal = 1
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error

"""
Move robot arm from one position to another
"""
def move_arm(pub_cmd, loop_rate, dest, vel, accel):

	global thetas, SPIN_RATE

	error = 0
	spin_count = 0
	at_goal = 0

	driver_msg = command()
	driver_msg.destination = dest
	driver_msg.v = vel
	driver_msg.a = accel
	driver_msg.io_0 = current_io_0
	pub_cmd.publish(driver_msg)

	loop_rate.sleep()

	while(at_goal == 0):

		if( abs(thetas[0]-driver_msg.destination[0]) < 0.0005 and \
			abs(thetas[1]-driver_msg.destination[1]) < 0.0005 and \
			abs(thetas[2]-driver_msg.destination[2]) < 0.0005 and \
			abs(thetas[3]-driver_msg.destination[3]) < 0.0005 and \
			abs(thetas[4]-driver_msg.destination[4]) < 0.0005 and \
			abs(thetas[5]-driver_msg.destination[5]) < 0.0005 ):

			at_goal = 1
			#rospy.loginfo("Goal is reached!")
		
		loop_rate.sleep()

		if(spin_count >  SPIN_RATE*5):

			pub_cmd.publish(driver_msg)
			rospy.loginfo("Just published again driver_msg")
			spin_count = 0

		spin_count = spin_count + 1

	return error

def move_block(pub_cmd, loop_rate, start_xw_yw_zw, target_xw_yw_zw, vel, accel):

    print(start_xw_yw_zw)
    start_dest = lab_invk(float(start_xw_yw_zw.x), float(start_xw_yw_zw.y), float(start_xw_yw_zw.z),0)
    rospy.sleep(0.5)
    print(start_dest)
    move_arm(pub_cmd, loop_rate, start_dest, vel, accel)
    gripper(pub_cmd, loop_rate, True)
    rospy.sleep(0.5)
    start_dest_high = lab_invk(float(start_xw_yw_zw.x), float(start_xw_yw_zw.y), float(.2), 0)
    move_arm(pub_cmd, loop_rate, start_dest_high, vel, accel)

    print(digital_in_0)
    if digital_in_0:
    	end_dest = lab_invk(float(target_xw_yw_zw.x), float(target_xw_yw_zw.y), float(target_xw_yw_zw.z),0)
	end_dest_high = lab_invk(float(target_xw_yw_zw.x), float(target_xw_yw_zw.y), float(.2), 0)
    	move_arm(pub_cmd, loop_rate, end_dest_high, vel, accel)
        rospy.sleep(0.5)
	move_arm(pub_cmd, loop_rate, end_dest, vel, accel)
	gripper(pub_cmd, loop_rate, False)
	rospy.sleep(0.5)
    else:
	print("Missing block")
	move_arm(pub_cmd, loop_rate, home, vel, accel)

    move_arm(pub_cmd, loop_rate, home, vel, accel)
    rospy.sleep(0.5)

    # global variable1
    # global variable2
    error = 0

    return error
"""
Program run from here
"""
def main():
	
	global home
	
	# Initialize ROS node
	rospy.init_node('project')
	parser = argparse.ArgumentParser(description='Which block(s) to pick up (1, 2, or 3)?')
	parser.add_argument('--block', nargs='+',type=int)
	args = parser.parse_args()
	block_number = args.block

    # Initialize publisher for ur3/command with buffer size of 10
	pub_command = rospy.Publisher('ur3/command', command, queue_size=10)

	# Initialize subscriber to ur3/position & ur3/gripper_input and callback fuction
	# each time data is published
	sub_position = rospy.Subscriber('ur3/position', position, position_callback)
	sub_input = rospy.Subscriber('ur3/gripper_input', gripper_input, input_callback)
	sub_hokuyo = rospy.Subscriber('spur/laser/scan', LaserScan, laser_callback)
	
  	spawn_block(0,False)	
	
	vel = 4.0
	accel = 4.0

	# Check if ROS is ready for operation
	while(rospy.is_shutdown()):
		print("ROS is shutdown!")

	# Initialize the rate to publish to ur3/command
	loop_rate = rospy.Rate(SPIN_RATE)
	loop = 0
	j = 1
	move_arm(pub_command, loop_rate, home, vel, accel)
	for i in block_number:
	   if not i == 1 and (i == 2 or i == 3):
	      block_world = hokuyo2world(range_min, range_max, angle_min, angle_max, angle_increment, ranges, i-loop, j)
	   elif i == 1:
	      block_world = hokuyo2world(range_min, range_max, angle_min, angle_max, angle_increment, ranges, i, j)
	   else:
	      print("Invalid argument. Enter 1 or 2")
	      sys.exit()
	   block_goal.x = -block_world.x - 0.3
	   block_goal.y = block_world.y
	   block_goal.z = block_world.z
	   move_block(pub_command, loop_rate, block_world, block_goal, vel, accel)
	   if not i == 3:
           	loop = loop + 1
	   j = j + 1

	rospy.loginfo("Destination is reached!")


if __name__ == '__main__':
	
	try:
		main()
    # When Ctrl+C is executed, it catches the exception
	except rospy.ROSInterruptException:
		pass
