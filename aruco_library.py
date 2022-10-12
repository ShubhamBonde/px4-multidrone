#!/usr/bin/env python3
############## Task1.1 - ArUco Detection ##############

import numpy as np
import cv2
import cv2.aruco as aruco
import sys
import math
import time

def detect_ArUco(img):
	## function to detect ArUco markers in the image using ArUco library
	## argument: img is the test image
	## return: dictionary named Detected_ArUco_markers of the format {ArUco_id_no : corners}, where ArUco_id_no indicates ArUco id and corners indicates the four corner position of the aruco(numpy array)
	## 		   for instance, if there is an ArUco(0) in some orientation then, ArUco_list can be like
	## 				{0: array([[315, 163],
	#							[319, 263],
	#							[219, 267],
	#							[215,167]], dtype=float32)}

    Detected_ArUco_markers = {}
    ## enter your code here ##
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters)
    
    Detected_ArUco_markers = {}
    
    for i in range(len(ids)):
    	Detected_ArUco_markers[ids[i][0]]=(corners[i])

    return Detected_ArUco_markers


def Calculate_orientation_in_degree(Detected_ArUco_markers):
	## function to calculate orientation of ArUco with respective to the scale mentioned in problem statement
	## argument: Detected_ArUco_markers  is the dictionary returned by the function detect_ArUco(img)
	## return : Dictionary named ArUco_marker_angles in which keys are ArUco ids and the values are angles (angles have to be calculated as mentioned in the problem statement)
	##			for instance, if there are two ArUco markers with id 1 and 2 with angles 120 and 164 respectively, the 
	##			function should return: {1: 120 , 2: 164}

	ArUco_marker_angles = {}
	## enter your code here ##

	for key in Detected_ArUco_markers:
		angle_in_rads = math.atan2(Detected_ArUco_markers[key][0][3][1]-Detected_ArUco_markers[key][0][0][1], 
										Detected_ArUco_markers[key][0][3][0]-Detected_ArUco_markers[key][0][0][0])
		angle_in_degs = 180 - math.degrees(angle_in_rads)
		ArUco_marker_angles[key] = angle_in_degs

	return ArUco_marker_angles	## returning the angles of the ArUco markers in degrees as a dictionary


def mark_ArUco(img,Detected_ArUco_markers,ArUco_marker_angles):
	## function to mark ArUco in the test image as per the instructions given in problem statement
	## arguments: img is the test image 
	##			  Detected_ArUco_markers is the dictionary returned by function detect_ArUco(img)
	##			  ArUco_marker_angles is the return value of Calculate_orientation_in_degree(Detected_ArUco_markers)
	## return: image namely img after marking the aruco as per the instruction given in problem statement	
	## enter your code here ##	
	font = cv2.FONT_HERSHEY_SIMPLEX	
	for key in Detected_ArUco_markers:

		img = cv2.circle(img, (int(Detected_ArUco_markers[key][0][0][0]), int(Detected_ArUco_markers[key][0][0][1])), 
						5, (125, 125, 125), -1) # gray
		img = cv2.circle(img, (int(Detected_ArUco_markers[key][0][1][0]), int(Detected_ArUco_markers[key][0][1][1])), 
						5, (0, 255, 0), -1) # green
		img = cv2.circle(img, (int(Detected_ArUco_markers[key][0][2][0]), int(Detected_ArUco_markers[key][0][2][1])), 
						5, (180, 105, 255), -1) # pink
		img = cv2.circle(img, (int(Detected_ArUco_markers[key][0][3][0]), int(Detected_ArUco_markers[key][0][3][1])), 
						5, (255, 255, 255), -1) # white
	
		# Getting the centers.
		cx = int((Detected_ArUco_markers[key][0][3][0] + Detected_ArUco_markers[key][0][1][0])/2)
		cy = int((Detected_ArUco_markers[key][0][3][1] + Detected_ArUco_markers[key][0][1][1])/2)
		l_centerX =  int((Detected_ArUco_markers[key][0][0][0] + Detected_ArUco_markers[key][0][1][0]) / 2)
		l_centerY = int((Detected_ArUco_markers[key][0][0][1] + Detected_ArUco_markers[key][0][1][1]) / 2)
		img = cv2.line(img, (cx, cy), (l_centerX, l_centerY), (255, 0 , 0), 3)
		img = cv2.circle(img, (int(cx),int(cy)), 5, (0, 0, 255), -1)
		img = cv2.putText(img, str(round(ArUco_marker_angles[key])), (int(Detected_ArUco_markers[key][0][3][0])-10, int(Detected_ArUco_markers[key][0][3][1])-20), font, 
		               1, (0, 255, 0), 2, cv2.LINE_AA)
		img = cv2.putText(img, str(key), (cx+12, cy+10), font, 1, (0, 0, 255), 2, cv2.LINE_AA)    
	
	return img	


