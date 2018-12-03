#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import Twist, Pose ,Quaternion
from sensor_msgs.msg import LaserScan ,Image, CameraInfo
from nav_msgs.msg import Odometry
from image_geometry import PinholeCameraModel
import math
import random
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import sys

LaserError = 0.05
LaserScan_ranges=[]
global VelocityMessage_publisher
global VelocityMessage
def getRangesFromLaser():    
    msg = rospy.wait_for_message("scan", LaserScan)
    for i in range(360):
        if i <= 15 or i > 335:
            if msg.ranges[i] >= LaserError:
		LaserScan_ranges.append(msg.ranges[i])	


def initVelocityMessage():
	global VelocityMessage_publisher
	global VelocityMessage
	VelocityMessage_publisher = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
   	VelocityMessage = Twist()

def stopSlowly():
		global VelocityMessage_publisher
		global VelocityMes
	        VelocityMessage.linear.x = 0.09
        	VelocityMessage_publisher.publish
        	rospy.sleep(0.2)
                VelocityMessage.linear.x = 0.08
        	
        	VelocityMessage_publisher.publish(VelocityMessage)
                rospy.sleep(0.2)
                VelocityMessage.linear.x = 0.07
        	
        	VelocityMessage_publisher.publish(VelocityMessage)
                rospy.sleep(0.2)
                VelocityMessage.linear.x = 0.06
 
         	VelocityMessage_publisher.publish(VelocityMessage)
                rospy.sleep(0.2)
                VelocityMessage.linear.x = 0.05
                
        	VelocityMessage_publisher.publish(VelocityMessage)
        	rospy.sleep(0.2)
                VelocityMessage.linear.x = 0.04
                
        	VelocityMessage_publisher.publish(VelocityMessage)
        	rospy.sleep(0.2)
                VelocityMessage.linear.x = 0.03
                
        	VelocityMessage_publisher.publish(VelocityMessage)
        	rospy.sleep(0.2)
                VelocityMessage.linear.x = 0.02
                
        	VelocityMessage_publisher.publish(VelocityMessage)
        	rospy.sleep(0.2)
                VelocityMessage.linear.x = 0.01
                
        	VelocityMessage_publisher.publish(VelocityMessage)
        	rospy.sleep(0.2)
                VelocityMessage.linear.x = 0.0
                
        	VelocityMessage_publisher.publish(VelocityMessage)
        	rospy.sleep(0.5)


        	VelocityMessage.linear.x = 0.0
        	
        	VelocityMessage_publisher.publish(VelocityMessage)
		rospy.sleep(0.5)

def forward():
    global VelocityMessage_publisher
    global VelocityMessage
    getRangesFromLaser()
    if min(LaserScan_ranges) > 0.6:
   	# Starts a new node
	initVelocityMessage()

   	#Since we are moving just in x-axis
    	VelocityMessage.angular.x = 0.0
    	VelocityMessage.angular.y = 0.0
   	VelocityMessage.angular.z = 0.0
    	VelocityMessage.linear.x = 0.1
    	VelocityMessage.linear.y = 0.0
   	VelocityMessage.linear.z = 0.0



   	while not rospy.is_shutdown():

       		#Setting the current time for distance calculus
       		start_time = rospy.Time.now().to_sec()
        	distancePassed = 0

 	        #Loop to move the turtle in an specified distance
  	        while(distancePassed < 0.40 and (not rospy.is_shutdown())):
           		#Publish the velocity
           		VelocityMessage_publisher.publish(VelocityMessage)
            		#Takes actual time to velocity calculus
           		now=rospy.Time.now().to_sec()
           		#Calculates distancePoseStamped
            		distancePassed = 0.1*(now-start_time)
       		#After the loop, stops the robot by braking slowly
		stopSlowly()
		#return 1 when there is no obstacle and forward succeeded
		return 1
    
    print("There is an obstacle in the way!")
    rospy.sleep(0.5)
    #returns 0 if there is an obstacle
    return 0
if __name__ == '__main__':
	rospy.init_node('node1', anonymous=True)
	forward()
