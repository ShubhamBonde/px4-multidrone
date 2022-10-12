#!/usr/bin/env python3


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames 
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name marker_detection which detects a moving ArUco marker.
This node publishes and subsribes the following topics:

	Subsriptions					Publications
	/camera/camera/image_raw			/marker_info
'''
from sensor_msgs.msg import Image
from task_1.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import rospy
import math
import cv2.aruco as aruco

#global i_d
#global centerx
#global centery
#global mydegrees

#i_d = 0
#centerx = 0
#centery = 0
#mydegrees = 0


class image_proc():

	# Initialise everything
	def __init__(self):
		self.i_d = []
		self.i_d_new = 0
		self.centerx = []
		self.centery = []
		self.centre_x_p = 0
		self.centre_y_p = 0
		self.mydegrees = 0
		rospy.init_node('marker_detection') #Initialise rosnode 
		
		# Making a publisher 
		
		self.marker_pub = rospy.Publisher('/marker_info', Marker, queue_size=10)
		
		# ------------------------Add other ROS Publishers here-----------------------------------------------------
	
        	# Subscribing to /camera/camera/image_raw

		self.image_sub = rospy.Subscriber("/camera/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
		
	        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        
		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()
		
		self.marker_msg=Marker()  # This will contain the message structure of message type task_1/Marker


	# Callback function of amera topic
	def image_callback(self, data):
	# Note: Do not make this function lenghty, do all the processing outside this callback function
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
			
# img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
			gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
			aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
			parameters = aruco.DetectorParameters_create()
			corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)

			Detected_ArUco_markers = {}

			for i in range(len(ids)):
				Detected_ArUco_markers[ids[i][0]]=(corners[i])


			self.i_d = []
			right_top = []
			left_top = []
			right_bottom = []
			left_bottom = []
			self.centerx = []
			self.centery = []
			centrex_l = []
			centrey_l = []
			 
			#print(Detected_ArUco_markers)
			for i in Detected_ArUco_markers:
				self.i_d.append(i)
				#print(i)
				left_top.append(Detected_ArUco_markers[i][0][0])
				right_top.append(Detected_ArUco_markers[i][0][1])
				right_bottom.append(Detected_ArUco_markers[i][0][2])
				left_bottom.append(Detected_ArUco_markers[i][0][3])
				

			# 
			#print(f'Left top: {left_top[0][1]}')
			angles = []


			for i in range(len(right_top)):
				myradians = math.atan2(left_bottom[i][1]-left_top[i][1], left_bottom[i][0]-left_top[i][0])
				self.mydegrees = 180 - math.degrees(myradians)
				#print(mydegrees)
				img = cv2.circle(self.img, (int(left_top[i][0]), int(left_top[i][1])), 5, (125, 125, 125), -1) # gray
				img = cv2.circle(self.img, (int(right_top[i][0]), int(right_top[i][1])), 5, (0, 255, 0), -1) # green
				img = cv2.circle(self.img, (int(right_bottom[i][0]), int(right_bottom[i][1])), 5, (180, 105, 255), -1) # pink
				img = cv2.circle(self.img, (int(left_bottom[i][0]), int(left_bottom[i][1])), 5, (255, 255, 255), -1) # white
				self.centre_x_p = int(abs((int(right_top[i][0]))+(int(left_bottom[i][0])))/2)
				self.centerx.append(self.centre_x_p)
				self.centre_y_p = int(abs((int(right_top[i][1]))+(int(left_bottom[i][1])))/2)
				self.centery.append(self.centre_y_p)
				 
				img = cv2.circle(self.img, (int(self.centerx[i]), int(self.centery[i])), 5, (0, 0, 255), -1) # white
				print("Centres are: ", self.centerx, self.centery)
				print(type(self.i_d))
				self.i_d_new = int(self.i_d[0])
				img = cv2.line(self.img, ((int(abs((int(left_top[i][0]))+(int(right_top[i][0])))/2)),(int(abs((int(left_top[i][1]))+(int(right_top[i][1])))/2))),(int(self.centerx[i]), int(self.centery[i])), (255,0,0), 2)
				self.publish_data()
				
			cv2.imshow("Window", self.img)
			cv2.waitKey(1)
				
		
		except CvBridgeError as e:
			print(e)
			
			
	def publish_data(self):
		self.marker_msg.id = self.i_d_new
		self.marker_msg.x = self.centre_x_p
		self.marker_msg.y = self.centre_y_p
		#
		self.marker_msg.z = 0
		self.marker_msg.roll = 0
		self.marker_msg.pitch = 0
		self.marker_msg.yaw = int(self.mydegrees)
		print("PUBLISHING")
		self.marker_pub.publish(self.marker_msg)
		rospy.loginfo(self.marker_msg)
		
if __name__ == '__main__':
    image_proc_obj = image_proc()
    
    image_proc_obj.publish_data()
    
    rospy.spin()
