#!/usr/bin/env python

#########################################################################################################
								   	 # IMPORTS
#########################################################################################################
from __future__ import division

from matplotlib.pyplot import box
import rospy
import math
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, ParamValue, State, WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, ParamSet, SetMode, SetModeRequest, WaypointClear, WaypointPush
from pymavlink import mavutil
from sensor_msgs.msg import NavSatFix, Imu, Image
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import math
import cv2.aruco as aruco
from gazebo_ros_link_attacher.srv import Gripper

from std_msgs.msg import *
from threading import Thread
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Image
from multiprocessing import Process, Array


########################################################################################################
											#GLOBALS
########################################################################################################

collect_gps = 0
gx = 100
gy = 100
box_loc_x = 100
box_loc_y = 100
init_pos = [-1, 61]
blue_truck_pos = [14.7, -7.4, 1.7]
red_truck_pos = [58.2, 64.75, 1.7]

########################################################################################################
#										SHARED MEMORY ARRAYS										   #
########################################################################################################

boxloc_setpoint_array = Array('i', 8)
box_ids = Array('i',4)

#########################################################################################################
								   	 # DRONECONTROL CLASS
#########################################################################################################
class DroneControl():
	def __init__(self, *args):
		rospy.loginfo("INITIATING EDRONE0")
		self.altitude = Altitude()
		self.extended_state = ExtendedState()
		self.global_position = NavSatFix()
		self.imu_data = Imu()
		self.home_position = HomePosition()
		self.local_position = PoseStamped()
		self.vel = Twist()
		self.mission_wp = WaypointList()
		self.state = State()
		self.mav_type = None
		self.img = np.empty([])
		self.bridge = CvBridge() 
		self.marker_msg=Marker()
		self.width = 0
		self.height = 0
		self.boxn = 0
		self.idn = 0
		self.pos = PoseStamped()
		self.radius = 0.1


#######################################################################################################
								# Position and velocity Publishers
#######################################################################################################


		self.pos_setpoint_pub = rospy.Publisher('edrone0/mavros/setpoint_position/local', PoseStamped, queue_size=10)

		self.vel_setpoint_pub = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel',
			Twist, queue_size=10)	
		self.velo = Twist()
		self.velo.linear.x = 2
		self.velo.linear.y = 2
		self.velo.linear.z = 2



#######################################################################################################
							# MULTITHREADING FOR FAILSAFE PREVENTION
#######################################################################################################

		# send setpoints in seperate thread to better prevent failsafe
		self.pos_thread = Thread(target=self.send_pos, args=())
		self.pos_thread.daemon = True
		self.pos_thread.start()

		self.vel_thread = Thread(target=self.send_vel, args=())
		self.vel_thread.daemon = True
		self.vel_thread.start()
		
		


		self.sub_topics_ready = {
			key: False
			for key in [
				'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
				'mission_wp', 'state', 'imu'
			]
		}
		
#########################################################################################################
											 # ROS services
#########################################################################################################
		service_timeout = 30
		rospy.loginfo("waiting for ROS services")
		try:
			rospy.wait_for_service('edrone0/mavros/param/get', service_timeout)
			rospy.wait_for_service('edrone0/mavros/param/set', service_timeout)
			rospy.wait_for_service('edrone0/mavros/cmd/arming', service_timeout)
			rospy.wait_for_service('edrone0/mavros/mission/push', service_timeout)
			rospy.wait_for_service('edrone0/mavros/mission/clear', service_timeout)
			rospy.wait_for_service('edrone0/mavros/set_mode', service_timeout)
			rospy.loginfo("ROS services are up")
		except rospy.ROSException:
			self.fail("failed to connect to services")
		self.get_param_srv = rospy.ServiceProxy('edrone0/mavros/param/get', ParamGet)
		self.set_param_srv = rospy.ServiceProxy('edrone0/mavros/param/set', ParamSet)
		self.set_arming_srv = rospy.ServiceProxy('edrone0/mavros/cmd/arming',
												 CommandBool)
		self.set_mode_srv = rospy.ServiceProxy('edrone0/mavros/set_mode', SetMode)
		self.wp_clear_srv = rospy.ServiceProxy('edrone0/mavros/mission/clear',
											   WaypointClear)
		self.wp_push_srv = rospy.ServiceProxy('edrone0/mavros/mission/push',
											  WaypointPush)

		# ROS subscribers
		self.alt_sub = rospy.Subscriber('edrone0/mavros/altitude', Altitude,
										self.altitude_callback)
		self.ext_state_sub = rospy.Subscriber('edrone0/mavros/extended_state',
											  ExtendedState,
											  self.extended_state_callback)
		self.global_pos_sub = rospy.Subscriber('edrone0/mavros/global_position/global',
											   NavSatFix,
											   self.global_position_callback)
		self.imu_data_sub = rospy.Subscriber('edrone0/mavros/imu/data',
											   Imu,
											   self.imu_data_callback)
		self.home_pos_sub = rospy.Subscriber('edrone0/mavros/home_position/home',
											 HomePosition,
											 self.home_position_callback)
		self.local_pos_sub = rospy.Subscriber('edrone0/mavros/local_position/pose',
											  PoseStamped, self.local_position_callback)
		self.vel_sub = rospy.Subscriber('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, self.vel_cb)
		self.mission_wp_sub = rospy.Subscriber(
			'edrone0/mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
		self.state_sub = rospy.Subscriber('edrone0/mavros/state', State ,
										  self.state_callback)

		# eDrone publisher
		print("Subscribing to Camera")
		self.image_sub = rospy.Subscriber('/edrone0/camera/image_raw', Image, self.image_callback)
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()

	def tearDown(self):
		self.log_topic_vars()

		
	

#########################################################################################################
									# CALLBACKS
#########################################################################################################

	def altitude_callback(self, data):
		self.altitude = data

		if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
			self.sub_topics_ready['alt'] = True

	def extended_state_callback(self, data):
		self.extended_state = data

		if not self.sub_topics_ready['ext_state']:
			self.sub_topics_ready['ext_state'] = True

	def global_position_callback(self, data):
		self.global_position = data

		if not self.sub_topics_ready['global_pos']:
			self.sub_topics_ready['global_pos'] = True

	def imu_data_callback(self, data):
		self.imu_data = data

		if not self.sub_topics_ready['imu']:
			self.sub_topics_ready['imu'] = True

	def home_position_callback(self, data):
		self.home_position = data

		if not self.sub_topics_ready['home_pos']:
			self.sub_topics_ready['home_pos'] = True

	def local_position_callback(self, data):
		global gx
		global gy
		self.local_position = data
		gx = data.pose.position.x
		gy = data.pose.position.y
		

		if not self.sub_topics_ready['local_pos']:
			self.sub_topics_ready['local_pos'] = True

	def vel_cb(self, data):
		pass


#########################################################################################################
									# IMAGE PROCESSING
#########################################################################################################

	def image_callback(self, data):

		global collect_gps
		global box_loc_x
		global box_loc_y

		global gx
		global gy

		global box_pos_x
		global box_pos_y
		
		rx = 0
		ry = 0

		"""callback required in the image subscriber and contains all the important logic"""
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image


			gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
			self.height = gray.shape[0]
			self.width = gray.shape[1]
			aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
			parameters = aruco.DetectorParameters_create()
			corners, self.ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

			Detected_ArUco_markers = {}

			### IF statement to avoid the Nonetype error message ###
			if self.ids: 
				for i in range(len(self.ids)):
					Detected_ArUco_markers[self.ids[i][0]]=(corners[i])
				

			# # Handle the data in lists
			i_d = []
			right_top = []
			left_top = []
			right_bottom = []
			left_bottom = []
			self.centerx = []
			centery = []
			for i in Detected_ArUco_markers:
				i_d.append(i)
				self.id = i
				left_top.append(Detected_ArUco_markers[i][0][0])
				right_top.append(Detected_ArUco_markers[i][0][1])
				right_bottom.append(Detected_ArUco_markers[i][0][2])
				left_bottom.append(Detected_ArUco_markers[i][0][3])
				
			# get the orientation
			for i in range(len(right_top)):
				myradians = math.atan2(left_bottom[i][1]-left_top[i][1], left_bottom[i][0]-left_top[i][0])
				mydegrees = 180 - math.degrees(myradians)
				self.yaw = mydegrees


				# draw coloured circles on each corner
				img = cv2.circle(self.img, (int(left_top[i][0]), int(left_top[i][1])), 5, (125, 125, 125), -1) # gray
				img = cv2.circle(self.img, (int(right_top[i][0]), int(right_top[i][1])), 5, (0, 255, 0), -1) # green
				img = cv2.circle(self.img, (int(right_bottom[i][0]), int(right_bottom[i][1])), 5, (180, 105, 255), -1) # pink
				img = cv2.circle(self.img, (int(left_bottom[i][0]), int(left_bottom[i][1])), 5, (255, 255, 255), -1) # white
				
				# find centers
				centre_x_p = int(abs((int(right_top[i][0]))+(int(left_bottom[i][0])))/2) #center x
				self.x = centre_x_p # will be our x
				self.centerx.append(centre_x_p)
				centre_y_p = int(abs((int(right_top[i][1]))+(int(left_bottom[i][1])))/2) # center y
				self.y = centre_y_p # will be our y
				centery.append(centre_y_p)
				img = cv2.circle(self.img, (int(self.centerx[i]), int(centery[i])), 5, (0, 0, 255), -1) # white
				print("Center X value is: ", self.centerx[i])
				if (int(self.centerx[i]) in range(175,210 ) and collect_gps==0) and self.idn <= 3:
					collect_gps = 1
					box_ids[self.idn] = i_d[0]
					self.idn+=1	

					rospy.loginfo("@@@@@@@@@@@@@@@@@@@@@}----Package Found-----{@@@@@@@@@@@@@@@@@@@@@@@")
					box_loc_x = math.floor(gx)-1
					box_loc_y = math.ceil(gy)
					boxloc_setpoint_array[self.boxn] = self.setpoint_processor_x(box_loc_x)
					self.boxn+=1
					boxloc_setpoint_array[self.boxn] = self.setpoint_processor_y(box_loc_y)
					self.boxn+=1
					rospy.loginfo(f"Appended ({self.setpoint_processor_x(box_loc_x)}, {self.setpoint_processor_y(box_loc_y)}, 4) into boxloc_setpoint_array")

				# Lables on aruco markers with orientation and id 
				img = cv2.line(self.img, ((int(abs((int(left_top[i][0]))+(int(right_top[i][0])))/2)),(int(abs((int(left_top[i][1]))+(int(right_top[i][1])))/2))),(int(self.centerx[i]), int(centery[i])), (255,0,0), 2)
				img = cv2.putText(img, str(round(mydegrees)), (int(left_bottom[i][0])-10, int(left_bottom[i][1])-20), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
				img = cv2.putText(img, str(i_d[0]), (centre_x_p+50, centre_y_p+10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)  
				# self.publish_data()
		except CvBridgeError as e:
			print("Error in photo")
			return	

		cv2.imshow("Window", self.img)
		cv2.waitKey(1)
		
		

			
		

	def mission_wp_callback(self, data):
		self.mission_wp = data

		if not self.sub_topics_ready['mission_wp']:
			self.sub_topics_ready['mission_wp'] = True

	def setpoint_processor_x(self, setpoint):
		if setpoint in range(7, 10):
			setpoint += (8 - setpoint)
		elif setpoint in range(19, 22):
			setpoint += (20 - setpoint)
		elif setpoint in range(4, 7):
			setpoint += (5 - setpoint)
		elif setpoint in range(14, 17):
			setpoint += (15 - setpoint)
		return setpoint
	
	def setpoint_processor_y(self, setpoint):
		if setpoint in range(16, 19):
			setpoint += (17 - setpoint)
		elif setpoint in range(24, 27):
			setpoint += (25 - setpoint)
		elif setpoint in range(28, 31):
			setpoint += (29 - setpoint)
		elif setpoint in range(48, 51):
			setpoint += (49 - setpoint)
		return setpoint

	def state_callback(self, data):
		self.state = data

		# mavros publishes a disconnected state message on init
		if not self.sub_topics_ready['state'] and data.connected:
			self.sub_topics_ready['state'] = True


#########################################################################################################
									# HELPER FUNCTIONS
#########################################################################################################
	def set_arm(self, arm, timeout):
		"""arm: True to arm or False to disarm, timeout(int): seconds"""
		# rospy.loginfo("setting FCU arm: {0}".format(arm))
		old_arm = self.state.armed
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		arm_set = False
		for i in range(timeout * loop_freq):
			if self.state.armed == arm:
				arm_set = True
				break
			else:
				try:
					res = self.set_arming_srv(arm)
					if not res.success:
						rospy.logerr("failed to send arm command")
				except rospy.ServiceException as e:
					rospy.logerr(e)

			try:
				rate.sleep()
			except rospy.ROSException as e:
				pass


	def set_mode(self, mode, timeout):
		"""mode: PX4 mode string, timeout(int): seconds"""
		old_mode = self.state.mode
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		mode_set = False
		for i in range(timeout * loop_freq):
			if self.state.mode == mode:
				mode_set = True
				break
			else:
				try:
					res = self.set_mode_srv(0, mode)  # 0 is custom mode
					if not res.mode_sent:
						rospy.logerr("failed to send mode command")
				except rospy.ServiceException as e:
					rospy.logerr(e)

			try:
				rate.sleep()
			except rospy.ROSException as e:
				pass



	def set_param(self, param_id, param_value, timeout):
		"""param: PX4 param string, ParamValue, timeout(int): seconds"""
		if param_value.integer != 0:
			value = param_value.integer
		else:
			value = param_value.real
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		param_set = False
		for i in range(timeout * loop_freq):
			try:
				res = self.set_param_srv(param_id, param_value)
				if res.success:
					rospy.loginfo("hi")
				break
			except rospy.ServiceException as e:
				rospy.logerr(e)

			try:
				rate.sleep()
			except rospy.ROSException as e:
				continue

	def wait_for_topics(self, timeout):
		"""wait for simulation to be ready, make sure we're getting topic info
		from all topics by checking dictionary of flag values set in callbacks,
		timeout(int): seconds"""
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		simulation_ready = False
		for i in range(timeout * loop_freq):
			if all(value for value in self.sub_topics_ready.values()):
				simulation_ready = True
				break

			try:
				rate.sleep()
			except rospy.ROSException as e:
				print(f"Exeption occured: {e}")
				pass


	def wait_for_landed_state(self, desired_landed_state, timeout, index):
		loop_freq = 10  # Hz
		rate = rospy.Rate(loop_freq)
		landed_state_confirmed = False
		for i in range(timeout * loop_freq):
			if self.extended_state.landed_state == desired_landed_state:
				landed_state_confirmed = True
				break

			try:
				rate.sleep()
			except rospy.ROSException as e:
				pass


	def wait_for_vtol_state(self, transition, timeout, index):
		"""Wait for VTOL transition, timeout(int): seconds"""
		loop_freq = 10  # Hz
		rate = rospy.Rate(loop_freq)
		transitioned = False
		for i in range(timeout * loop_freq):
			if transition == self.extended_state.vtol_state:
				transitioned = True
				break

			try:
				rate.sleep()
			except rospy.ROSException as e:
				pass


	def wait_for_mav_type(self, timeout):
		"""Wait for MAV_TYPE parameter, timeout(int): seconds"""
		loop_freq = 1  # Hz
		rate = rospy.Rate(loop_freq)
		res = False
		for i in range(timeout * loop_freq):
			try:
				res = self.get_param_srv('MAV_TYPE')
				if res.success:
					self.mav_type = res.value.integer
					break
			except rospy.ServiceException as e:
				rospy.logerr(e)

			try:
				rate.sleep()
			except rospy.ROSException as e:
				pass

	def log_topic_vars(self):
		pass

	#
	# Helper methods
	#
	def send_pos(self):


		rate = rospy.Rate(10)  # Hz
		self.pos.header = Header()
		self.pos.header.frame_id = "base_footprint"

		while not rospy.is_shutdown():
			self.pos.header.stamp = rospy.Time.now()
			self.pos_setpoint_pub.publish(self.pos)
			self.vel_setpoint_pub.publish(self.velo)
 
			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	def send_vel(self):
		print("Set Velocity")
		rate = rospy.Rate(10)  # Hz
		self.pos.header = Header()
		self.pos.header.frame_id = "base_footprint"

		while not rospy.is_shutdown():
			self.pos.header.stamp = rospy.Time.now()
			self.vel_setpoint_pub.publish(self.velo)
			
 
			try:  # prevent garbage in console output when thread is killed
				rate.sleep()
			except rospy.ROSInterruptException:
				pass

	def is_at_position(self, x, y, z, offset):
		"""offset: meters"""
		rospy.logdebug(
			"current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
				self.local_position.pose.position.x, self.local_position.pose.
				position.y, self.local_position.pose.position.z))

		desired = np.array((x, y, z))
		pos = np.array((self.local_position.pose.position.x,
						self.local_position.pose.position.y,
						self.local_position.pose.position.z))
		return np.linalg.norm(desired - pos) < offset

	def reach_position(self, x, y, z, xv, yv, zv, timeout):
		"""timeout(int): seconds"""
		
		self.pos.pose.position.x = x
		self.pos.pose.position.y = y
		self.pos.pose.position.z = z
		self.vel_setpoint_pub.publish(self.velo)

		yaw_degrees = 0  # North
		yaw = math.radians(yaw_degrees)
		quaternion = quaternion_from_euler(0, 0, yaw)
		self.pos.pose.orientation = Quaternion(*quaternion)

		loop_freq = 2  # Hz
		rate = rospy.Rate(loop_freq)
		reached = False
		for i in range(timeout * loop_freq):
			if self.is_at_position(self.pos.pose.position.x,
								   self.pos.pose.position.y,
								   self.pos.pose.position.z, self.radius):
				reached = True
				break

			try:
				rate.sleep()
			except rospy.ROSException as e:
				a = 20


'''/////////////////////////////////////////////////////////////////////////////////////////////////////////////'''
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
####################################################################################################################
											# DRONE 2 CONTROLLER CLASS
####################################################################################################################
#@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
'''/////////////////////////////////////////////////////////////////////////////////////////////////////////////'''

collect_gps2 = 0
gx2 = 100
gy2 = 100
box_loc_x2 = 100
box_loc_y2 = 100


class DroneControl2():
	def __init__(self, *args):
		rospy.loginfo("INITIATING EDRONE1")
		self.altitude2 = Altitude()
		self.extended_state2 = ExtendedState()
		self.global_position2 = NavSatFix()
		self.imu_data2 = Imu()
		self.home_position2 = HomePosition()
		self.local_position2 = PoseStamped()
		self.vel2 = Twist()
		self.mission_wp2 = WaypointList()
		self.state2 = State()
		self.mav_type2 = None
		self.img2 = np.empty([])
		self.bridge2 = CvBridge() 
		self.marker_msg2=Marker()
		self.width2 = 0
		self.height2 = 0
		self.boxloc_setpoint_array2= []
		self.box_ids2 = []
		self.pos2 = PoseStamped()
		self.radius2 = 0.08
		self.ready_to_pick = False


#######################################################################################################
								# Position and velocity Publishers
#######################################################################################################


		self.pos_setpoint_pub2 = rospy.Publisher('edrone1/mavros/setpoint_position/local', PoseStamped, queue_size=10)

		self.vel_setpoint_pub2 = rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel',
			Twist, queue_size=10)	
		self.velo2 = Twist()
		self.velo2.linear.x = 2
		self.velo2.linear.y = 2
		self.velo2.linear.z = 2



#######################################################################################################
							# MULTITHREADING FOR FAILSAFE PREVENTION
#######################################################################################################

		# send setpoints in seperate2 thread to better prevent failsafe
		self.pos_thread2 = Thread(target=self.send_pos2, args=())
		self.pos_thread2.daemon = True
		self.pos_thread2.start()

		self.vel_thread2 = Thread(target=self.send_vel2, args=())
		self.vel_thread2.daemon = True
		self.vel_thread2.start()
		
		


		self.sub_topics_ready2 = {
			key: False
			for key in [
				'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
				'mission_wp', 'state', 'imu'
			]
		}
		
#########################################################################################################
											 # ROS services
#########################################################################################################
		service_timeout2 = 30
		rospy.loginfo("waiting for ROS services")
		try:
			rospy.wait_for_service('edrone1/mavros/param/get', service_timeout2)
			rospy.wait_for_service('edrone1/mavros/param/set', service_timeout2)
			rospy.wait_for_service('edrone1/mavros/cmd/arming', service_timeout2)
			rospy.wait_for_service('edrone1/mavros/mission/push', service_timeout2)
			rospy.wait_for_service('edrone1/mavros/mission/clear', service_timeout2)
			rospy.wait_for_service('edrone1/mavros/set_mode', service_timeout2)
			rospy.wait_for_service('/edrone1/activate_gripper', service_timeout2)
			rospy.loginfo("ROS services are up")
		except rospy.ROSException:
			rospy.loginfo("failed to connect to services")
		self.get_param_srv2 = rospy.ServiceProxy('edrone1/mavros/param/get', ParamGet)
		self.set_param_srv2 = rospy.ServiceProxy('edrone1/mavros/param/set', ParamSet)
		self.set_arming_srv2 = rospy.ServiceProxy('edrone1/mavros/cmd/arming',
												 CommandBool)
		self.set_mode_srv2 = rospy.ServiceProxy('edrone1/mavros/set_mode', SetMode)
		self.set_gripper_1 = rospy.ServiceProxy("/edrone1/activate_gripper", Gripper)


		# ROS subscribers
		self.alt_sub2 = rospy.Subscriber('edrone1/mavros/altitude', Altitude,
										self.altitude_callback2)
		self.ext_state_sub2 = rospy.Subscriber('edrone1/mavros/extended_state',
											  ExtendedState,
											  self.extended_state_callback2)
		self.global_pos_sub2 = rospy.Subscriber('edrone1/mavros/global_position/global',
											   NavSatFix,
											   self.global_position_callback2)
		self.imu_data_sub2 = rospy.Subscriber('edrone1/mavros/imu/data',
											   Imu,
											   self.imu_data_callback2)
		self.home_pos_sub2 = rospy.Subscriber('edrone1/mavros/home_position/home',
											 HomePosition,
											 self.home_position_callback2)
		self.local_pos_sub2 = rospy.Subscriber('edrone1/mavros/local_position/pose',
											  PoseStamped, self.local_position_callback2)
		self.vel_sub2 = rospy.Subscriber('edrone1/mavros/setpoint_velocity/cmd_vel', Twist, self.vel_cb2)
		self.state_sub2 = rospy.Subscriber('edrone1/mavros/state', State ,
										  self.state_callback2)

		rospy.loginfo("Subscribers are up")
	def tearDown2(self):
		self.log_topic_vars2()

		
	

#########################################################################################################
									# CALLBACKS
#########################################################################################################

	def gripper_act(self, val):
		self.set_gripper_1(val)

	def altitude_callback2(self, data):
		self.altitude2 = data

		# amsl has been observed to be nan while other fields are valid
		if not self.sub_topics_ready2['alt'] and not math.isnan(data.amsl):
			self.sub_topics_ready2['alt'] = True

	def extended_state_callback2(self, data):
		self.extended_state2 = data

		if not self.sub_topics_ready2['ext_state']:
			self.sub_topics_ready2['ext_state'] = True

	def global_position_callback2(self, data):
		self.global_position2 = data

		if not self.sub_topics_ready2['global_pos']:
			self.sub_topics_ready2['global_pos'] = True

	def imu_data_callback2(self, data):
		self.imu_data2 = data

		if not self.sub_topics_ready2['imu']:
			self.sub_topics_ready2['imu'] = True

	def home_position_callback2(self, data):
		self.home_position2 = data

		if not self.sub_topics_ready2['home_pos']:
			self.sub_topics_ready2['home_pos'] = True

	def local_position_callback2(self, data):
		global gx2
		global gy2
		self.local_position2 = data
		gx2 = data.pose.position.x
		gy2 = data.pose.position.y

		

		if not self.sub_topics_ready2['local_pos']:
			self.sub_topics_ready2['local_pos'] = True

	def vel_cb2(self, data):
		pass


#########################################################################################################
									# IMAGE PROCESSING
#########################################################################################################


	def state_callback2(self, data):
		self.state2 = data

		# mavros publishes a disconnected state message on init
		if not self.sub_topics_ready2['state'] and data.connected:
			self.sub_topics_ready2['state'] = True


#########################################################################################################
									# HELPER FUNCTIONS
#########################################################################################################
	def set_arm2(self, arm, timeout):
		"""arm: True to arm or False to disarm, timeout(int): seconds"""
		# rospy.loginfo("setting FCU arm: {0}".format(arm))
		old_arm2 = self.state2.armed
		loop_freq2 = 1  # Hz
		rate2 = rospy.Rate(loop_freq2)
		arm_set = False
		for i in range(timeout * loop_freq2):
			if self.state2.armed == arm:
				arm_set = True
				break
			else:
				try:
					res2 = self.set_arming_srv2(arm)
					if not res2.success:
						rospy.logerr("failed to send arm command")
				except rospy.ServiceException as e:
					rospy.logerr(e)

			try:
				rate2.sleep()
			except rospy.ROSException as e2:
				pass


	def set_mode2(self, mode, timeout):
		"""mode: PX4 mode string, timeout(int): seconds"""
		# rospy.loginfo("setting FCU mode: {0}".format(mode))
		old_mode2 = self.state2.mode
		loop_freq2 = 1  # Hz
		rate2 = rospy.Rate(loop_freq2)
		mode_set2 = False
		for i in range(timeout * loop_freq2):
			if self.state2.mode == mode:
				mode_set2 = True
				break
			else:
				try:
					res2 = self.set_mode_srv2(0, mode)  # 0 is custom mode
					if not res2.mode_sent:
						rospy.logerr("failed to send mode command")
				except rospy.ServiceException as e:
					rospy.logerr(e)

			try:
				rate2.sleep()
			except rospy.ROSException as e2:
				pass



	def set_param2(self, param_id, param_value, timeout):
		"""param: PX4 param string, ParamValue, timeout(int): seconds"""
		if param_value.integer != 0:
			value = param_value.integer
		else:
			value = param_value.real
		loop_freq2 = 1  # Hz
		rate2 = rospy.Rate(loop_freq2)
		param_set2 = False
		for i in range(timeout * loop_freq2):
			try:
				res2 = self.set_param_srv2(param_id, param_value)
				if res2.success:
					rospy.loginfo("hi")
				break
			except rospy.ServiceException as e:
				rospy.logerr(e)

			try:
				rate2.sleep()
			except rospy.ROSException as e2:
				continue

	def wait_for_topics2(self, timeout):
		"""wait for simulation to be ready, make sure we're getting topic info
		from all topics by checking dictionary of flag values set in callbacks,
		timeout(int): seconds"""
		loop_freq2 = 1  # Hz
		rate2 = rospy.Rate(loop_freq2)
		simulation_ready2 = False
		for i in range(timeout * loop_freq2):
			if all(value for value in self.sub_topics_ready2.values()):
				simulation_ready2 = True
				break

			try:
				rate2.sleep()
			except rospy.ROSException as e2:
				print(f"Exeption occured: {e2}")
				pass


	def wait_for_landed_state2(self, desired2_landed_state, timeout, index):
		loop_freq2 = 10  # Hz
		rate2 = rospy.Rate(loop_freq2)
		landed_state_confirmed = False
		for i in range(timeout * loop_freq2):
			if self.extended_state2.landed_state == desired2_landed_state:
				landed_state_confirmed = True
				break

			try:
				rate2.sleep()
			except rospy.ROSException as e2:
				pass


	def wait_for_vtol_state2(self, transition, timeout, index):
		"""Wait for VTOL transition, timeout(int): seconds"""

		loop_freq2 = 10  # Hz
		rate2 = rospy.Rate(loop_freq2)
		transitioned2 = False
		for i in range(timeout * loop_freq2):
			if transition == self.extended_state2.vtol_state:
					box_loc_y = math.ceil(gy)

			try:
				rate2.sleep()
			except rospy.ROSException as e2:
				pass


	def wait_for_mav_type2(self, timeout):
		"""Wait for MAV_TYPE parameter, timeout(int): seconds"""
		# rospy.loginfo("waiting for MAV_TYPE")
		loop_freq2 = 1  # Hz
		rate2 = rospy.Rate(loop_freq2)
		res2 = False
		for i in range(timeout * loop_freq2):
			try:
				res2 = self.get_param_srv2('MAV_TYPE')
				if res2.success:
					self.mav_type2 = res2.value.integer

					break
			except rospy.ServiceException as e:
				rospy.logerr(e)

			try:
				rate2.sleep()
			except rospy.ROSException as e2:
				pass

	def log_topic_vars2(self):
		pass


	def send_pos2(self):


		rate2 = rospy.Rate(10)  # Hz
		self.pos2.header = Header()
		self.pos2.header.frame_id = "base_footprint"

		while not rospy.is_shutdown():
			self.pos2.header.stamp = rospy.Time.now()
			self.pos_setpoint_pub2.publish(self.pos2)
			self.vel_setpoint_pub2.publish(self.velo2)
 
			try:  # prevent garbage in console output when thread is killed
				rate2.sleep()
			except rospy.ROSInterruptException:
				pass

	def send_vel2(self):
		print("Set Velocity")
		rate2 = rospy.Rate(10)  # Hz
		self.pos2.header = Header()
		self.pos2.header.frame_id = "base_footprint"

		while not rospy.is_shutdown():
			self.pos2.header.stamp = rospy.Time.now()
			self.vel_setpoint_pub2.publish(self.velo2)
			
 
			try:  # prevent garbage in console output when thread is killed
				rate2.sleep()
			except rospy.ROSInterruptException:
				pass

	def is_at_position2(self, x, y, z, offset):
		"""offset: meters"""
		rospy.logdebug(
			"current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
				self.local_position2.pose.position.x, self.local_position2.pose.
				position.y, self.local_position2.pose.position.z))

		desired2 = np.array((x, y, z))
		pos2 = np.array((self.local_position2.pose.position.x,
						self.local_position2.pose.position.y,
						self.local_position2.pose.position.z))
		return np.linalg.norm(desired2 - pos2) < offset
	
	def change_rad(self, val):
		self.radius2 = val
	

	def reach_position2(self, x, y, z, xv, yv, zv, timeout):
		"""timeout(int): seconds"""
		
		self.pos2.pose.position.x = x
		self.pos2.pose.position.y = y
		self.pos2.pose.position.z = z


		self.vel_setpoint_pub2.publish(self.velo2)

		yaw_degrees2 = 0  # North
		yaw2 = math.radians(yaw_degrees2)
		quaternion = quaternion_from_euler(0, 0, yaw2)
		self.pos2.pose.orientation = Quaternion(*quaternion)

		loop_freq2 = 2  # Hz
		rate2 = rospy.Rate(loop_freq2)
		reached = False
		for i in range(timeout * loop_freq2):
			if self.is_at_position2(self.pos2.pose.position.x,
								   self.pos2.pose.position.y,
								   self.pos2.pose.position.z, self.radius2):
	
				reached = True
				break

			try:
				rate2.sleep()
			except rospy.ROSException as e2:
				print("Setting vel error")








def deploy_scanner():
	# rospy.init_node('scanner', anonymous=True)

	

	offboard = DroneControl()

	offboard.wait_for_topics(60)
	offboard.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
								10, -1)

	offboard.log_topic_vars()
	# exempting failsafe from lost RC to allow offboard
	rcl_except = ParamValue(1<<2, 0.0)
	offboard.set_param("COM_RCL_EXCEPT", rcl_except, 5)
	offboard.set_mode("OFFBOARD", 5)
	offboard.set_arm(True, 5)

	

	rospy.loginfo("run mission")

	positions = ((-1, 1, 3), (-1, 16.2, 3), (61, 16.2, 3), (61, 24.2, 3), (-1, 24.2, 3), (-1, 28.2, 3), (61, 28.2, 3), (61, 48.2, 3), (-1, 48.2, 3), (-1, 1, 3), (-1, 1, 0))

	for i in range(len(positions)):
		# rospy.sleep(5000)
		print("In Position ", i)
		global collect_gps
		collect_gps = 0
		offboard.reach_position(positions[i][0], positions[i][1], positions[i][2], 0.1, 0.1, 0.1, 60)
	
	offboard.set_mode("AUTO.LAND", 5)
	offboard.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
								45, 0)
	offboard.set_arm(False, 5)


def deploy_picker(ids, setpoints_arr):



	offboard2 = DroneControl2()

	offboard2.wait_for_topics2(60)
	offboard2.wait_for_landed_state2(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
								10, -1)

	offboard2.log_topic_vars2()
	# exempting failsafe from lost RC to allow offboard2
	rcl_except = ParamValue(1<<2, 0.0)
	offboard2.set_param2("COM_RCL_EXCEPT", rcl_except, 5)
	offboard2.set_mode2("OFFBOARD", 5)
	offboard2.set_arm2(True, 5)


	
	drone_1_x = -1
	drone_1_y = 61
	x = 0
	y = 1
	rospy.loginfo("RUNNING PICKER")

	positions = ((0, 0, 5))
	offboard2.reach_position2((drone_1_x)-(-1), (drone_1_y)-(61), 5, 0.1, 0.1, 0.1, 60)

	for i in range(len(box_ids[:])):
		current_x = gx2
		current_y = gy2
		print("Current X loc = ", current_x)
		print("Current Y loc = ", current_y)
		print("In Position ", i)
		offboard2.reach_position2(boxloc_setpoint_array[:][x]-drone_1_x, boxloc_setpoint_array[:][y]-drone_1_y, 5, 0.1, 0.1, 0.1, 100)
		offboard2.reach_position2(boxloc_setpoint_array[:][x]-drone_1_x, boxloc_setpoint_array[:][y]-drone_1_y, 0, 0.1, 0.1, 0.1, 20)
		x += 2
		y += 2
		offboard2.gripper_act(True)

		if box_ids[:][i]==2:
			offboard2.change_rad(1)
			offboard2.reach_position2(blue_truck_pos[0]-drone_1_x, blue_truck_pos[1]-drone_1_y, 5, 0.1, 0.1, 0.1, 30)
			offboard2.reach_position2(blue_truck_pos[0]-drone_1_x, blue_truck_pos[1]-drone_1_y, blue_truck_pos[2], 0.1, 0.1, 0.1, 15)
			offboard2.gripper_act(False)
			blue_truck_pos[1] = blue_truck_pos[1] + 1.23
			offboard2.reach_position2(blue_truck_pos[0]-drone_1_x, blue_truck_pos[1]-drone_1_y, 5, 0.1, 0.1, 0.1, 30)
			offboard2.change_rad(0.08)

		if box_ids[:][i]==1:
			offboard2.change_rad(1)
			offboard2.reach_position2(red_truck_pos[0]-drone_1_x, red_truck_pos[1]-drone_1_y, 5, 0.1, 0.1, 0.1, 30)
			offboard2.reach_position2(red_truck_pos[0]-drone_1_x, red_truck_pos[1]-drone_1_y, red_truck_pos[2], 0.1, 0.1, 0.1, 15)
			offboard2.gripper_act(False)
			red_truck_pos[1] = red_truck_pos[1] + 1.23
			offboard2.reach_position2(red_truck_pos[0]-drone_1_x, red_truck_pos[1]-drone_1_y, 5, 0.1, 0.1, 0.1, 30)
			offboard2.change_rad(0.08)
	
	offboard2.reach_position2((drone_1_x)-(-1), (drone_1_y)-(61), 5, 0.1, 0.1, 0.1, 30)
	offboard2.reach_position2((drone_1_x)-(-1), (drone_1_y)-(61), 0, 0.1, 0.1, 0.1, 30)



	offboard2.set_mode2("AUTO.LAND", 5)
	offboard2.wait_for_landed_state2(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
								45, 0)
	offboard2.set_arm2(False, 5)

#########################################################################################################
									# MAIN FUNCTION
#########################################################################################################

if __name__ == "__main__":
	rospy.init_node('flight_control', anonymous=True)
	scanner = Thread(target = deploy_scanner, args=())
	scanner.daemon = True
	picker = Thread(target = deploy_picker, args = (box_ids, boxloc_setpoint_array))
	picker.daemon = True
	scanner.start()
	picker.start()
	scanner.join()
	picker.join()
	print(f"box_setpoints: {boxloc_setpoint_array[:]}")
	print(f"box_ids: {box_ids[:]}")
