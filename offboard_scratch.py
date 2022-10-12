#!/usr/bin/env python

import rospy
from geometry_msgs.msg import *
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
import numpy as np

current_state = State()
current_pose = PoseStamped()
def state_cb(msg):
	global current_state
	current_state = msg

def local_pos_callback(data):
	global gx
	global gy
	global gz
	gx = data.pose.position.x
	gy = data.pose.position.y
	gz = data.pose.position.z

def is_at_position(x, y, z, offset):
        """offset: meters"""
        desired = np.array((x, y, z))
        pos = np.array((gx,gy,gz))
        return np.linalg.norm(desired - pos) < offset


if __name__=="__main__":
	# global current_state
	rospy.init_node('offb_node', anonymous=True)	
	rospy.Subscriber("mavros/state", State, state_cb)
	local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
	local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)

	print("Publisher and Subscriber Created")
	arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
	set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)
	rospy.Subscriber(
                        'mavros/local_position/pose',
                        PoseStamped,
                        local_pos_callback)
	print("Clients Created")
	rate = rospy.Rate(20)
	
	while(not current_state.connected):
		print(current_state.connected)
		rate.sleep()
	rospy.loginfo(f"Current Mode : {current_state.mode}")
	
	print("Creating pose")
	pose0 = PoseStamped()
	#set position here
	pose0.pose.position.x = 0
	pose0.pose.position.y = 0
	pose0.pose.position.z = 0

	pose1 = PoseStamped()
	#set position here
	pose1.pose.position.x = 0
	pose1.pose.position.y = 0
	pose1.pose.position.z = 10

	pose2 = PoseStamped()
	#set position here
	pose2.pose.position.x = 10
	pose2.pose.position.y = 0
	pose2.pose.position.z = 10
	
	pose3 = PoseStamped()
	#set position here
	pose3.pose.position.x = 10
	pose3.pose.position.y = 10
	pose3.pose.position.z = 10
	
	pose4 = PoseStamped()
	#set4position here
	pose4.pose.position.x = 0
	pose4.pose.position.y = 10
	pose4.pose.position.z = 10

	vel = Twist()
	vel.linear.x = 5
	vel.linear.y = 5
	vel.linear.z = 5
	
	for i in range(100):
		local_pos_pub.publish(pose0)
		#rate.sleep()
		
	print("Creating Objects for services")
	offb_set_mode = SetMode()
	offb_set_mode.custom_mode = "OFFBOARD"
	arm_cmd = CommandBool()
	arm_cmd.value = True
	
	last_request = rospy.Time.now()
	
	while not rospy.is_shutdown():
		#print(current_state)
		if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
			resp1 = set_mode_client(0,offb_set_mode.custom_mode)
			if resp1.mode_sent:
				rospy.loginfo(f"Offboard mode enabled")
			last_request = rospy.Time.now()
		elif (not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0))):
			arm_client_1 = arming_client(arm_cmd.value)
			if arm_client_1.success:
				print("ARMED!!!")
			last_request = rospy.Time.now()
		
		# Give the next setpoint as 10m,0m,10m followed by 10m,10m,10m followed by 0m,10m,10m and then back to 0m,0m,10m
		poses = ((0,0,10),(10,0,10),(10,10,10),(0,10,10),(0,0,10))
		# for i in range(len(poses)):
		# 	pass
		# rospy.loginfo(is_at_position(10,0,10,10))
		if is_at_position(0,0,0,10):
			local_pos_pub.publish(pose1)
		if is_at_position(10,0,10, 10):
			local_pos_pub.publish(pose2)
		if is_at_position(10,10,10,10):
			local_pos_pub.publish(pose3)
		if is_at_position(0,10,10,10):
			local_pos_pub.publish(pose4)
		if is_at_position(0,0,0,0):
			local_pos_pub.publish(pose0)

		

		# if is_at_position(0,10,10):
		# 	local_pos_pub.publish(pose1)
		# rospy.loginfo("INTIAL")
		# # local_pos_pub.publish(pose2)
		# # local_pos_pub.publish(pose3)
		# # local_pos_pub.publish(pose4)
		# if round(gz) == 10 and round(gy) == 0 and round(gx) == 0:
		# 	local_pos_pub.publish(pose2)
		# elif round(gz) == 10:
		# 	local_pos_pub.publish(pose3)
		# elif round(gy) == 10 and round(gx) == 10 and round(gz) == 10:
		# 	local_pos_pub.publish(pose4)
		# elif round(gx) == 0 and round(gy) == 10 and round(gz) == 10:
		# 	local_pos_pub.publish(pose1)
		
		
		# elif gx >= 9.9 and gx <= 10 and gy >= 9.9 and gy <= 10 and gz >= 9.9 and gz <= 10:
		# 	local_pos_pub.publish(pose4)
		local_vel_pub.publish(vel)
		rospy.loginfo(f"Gx = {round(gx)}, Gy = {round(gy)} , Gz = {round(gz)}")
		
		#print current_state
		rate.sleep()
	 