#!/usr/bin/env python
import rospy
import tf
import math
import sys
from geometry_msgs.msg import Twist, Pose ,Quaternion
from sensor_msgs.msg import LaserScan ,Image, CameraInfo
from nav_msgs.msg import Odometry
from image_geometry import PinholeCameraModel
import random
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String


PI = 3.1415926535897

ranges= {'blue': ([80,0,0],[255,60,60]),
                  'red': ([0,0,80],[30,30,255]),
                  'green' : ([0,80,0],[80,255,60])}
ColoredObjDist = None
wayClear = True
#############################################################################################option 1
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

def MoveAhead():
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
		#return 1 when there is no obstacle and MoveAhead succeeded
		return 1
    
    print("There is an obstacle in the way!")
    rospy.sleep(0.5)
    #returns 0 if there is an obstacle
    return 0

##########################################################################################################start option 3
def prod(v1, v2):
  return sum((a*b) for a, b in zip(v1, v2))


def checkIfColorIsShowed(color):

    image_msg = rospy.wait_for_message("camera/image_raw", Image)
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_msg, "bgr8")
    gaus_blur = cv2.GaussianBlur(cv_image,(3,3),0)
    


    lower = np.array(ranges[color][0], dtype = "uint8")
    upper = np.array(ranges[color][1], dtype = "uint8")    
    mask = cv2.inRange(gaus_blur, lower, upper)
    kercl = np.ones((20,20))    
    kerop = np.ones((5,5))
    maskOpen = cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kerop)
    final_mask = cv2.morphologyEx(maskOpen,cv2.MORPH_CLOSE,kercl)
    
    _,cont,h = cv2.findContours(final_mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)
    return cont



def FindDist(color):
    
    cont = checkIfColorIsShowed(color)

    if cont == []:
        rospy.sleep(0.5)

        print "None"
        return None
    cnt = cont[0]
    M = cv2.moments(cnt)
    center_x = int(M['m10']/M['m00'])
    center_y = 400
    
    imgCntr = (400,400)
    camInfo = rospy.wait_for_message("camera/camera_info", CameraInfo)
    camera= PinholeCameraModel()
    camera.fromCameraInfo(camInfo)
    cenRay = camera.projectPixelTo3dRay(imgCntr)
    pixRay = camera.projectPixelTo3dRay((center_x,center_y))

 

    angle = math.acos(prod(pixRay, cenRay) / (math.sqrt(prod(pixRay, pixRay))) * (math.sqrt(prod(cenRay, cenRay))))

    dg = int(math.degrees(angle))

    
    if center_x>= imgCntr[0]:
        dg = 360-dg

   
    LaserMessage = rospy.wait_for_message("scan", LaserScan)
    return retrunResult(LaserMessage,dg)

def retrunResult(LaserMessage,dg):
    print "the distance is "
    if(dg!= 360):
        print (LaserMessage.ranges[dg])
        rospy.sleep(0.5)
        return (dg,LaserMessage.ranges[dg])
    else:
       print (LaserMessage.ranges[0])
       rospy.sleep(0.5)
       return (dg,LaserMessage.ranges[0])
  ################################################################################################################### start option 4

def turnAround(alpha,clockwise):
    turn_publisher = rospy.Publisher('/cmd_vel', Twist,queue_size=10)
    turn_msg = Twist()

    #turning speed
    turn_msg.linear.x = 0.0
    turn_msg.linear.y = 0.0
    turn_msg.linear.z = 0.0
    turn_msg.angular.x = 0.0
    turn_msg.angular.y = 0.0
    if clockwise:
        turn_msg.angular.z = -1.0
    else:
        turn_msg.angular.z = 1.0
    current_angle = 0.0
    relative_angle = (float(alpha))*2*PI/360

    while not rospy.is_shutdown():
        
        t0 = rospy.Time.now().to_sec()

        while((current_angle <= relative_angle) and (not rospy.is_shutdown())):
            turn_publisher.publish(turn_msg)
            t1 = rospy.Time.now().to_sec()
            current_angle = 1.0*(t1-t0)
        #stoping the robot from turning
        turn_msg.angular.z = 0.0
        turn_publisher.publish(turn_msg)
        rospy.sleep(0.5)
        return
    
def turnUntilFound(color):
    global ColoredObjDist
    global wayClear
    found = False
    while not found:
        for i in range(4):
            turnAround(90,True)
            ColoredObjDist = FindDist(color)
            if ColoredObjDist is not None:
                found = True
                break
        
        if not found:
            wayClear =  MoveAhead()
            while not wayClear:
                turnAround(90,True)
                wayClear =  MoveAhead()
                if(wayClear):
                    FindObjWithColor(color)
                    return
    


def FindObjWithColor(color):

      
    global ColoredObjDist
    global wayClear
    ColoredObjDist = None
    wayClear = True
    turnUntilFound(color)

    if ColoredObjDist[0] == 0:
        turnAround(ColoredObjDist[0],True)
    else:
        if 360-ColoredObjDist[0]>180:
            turnAround(ColoredObjDist[0],False)
        else:
            turnAround(360-ColoredObjDist[0],True)
    while ColoredObjDist[1]>1:
        wayClear =  MoveAhead()
        if not wayClear:
            turnAround(90,True)
            wayClear =  MoveAhead()
            if(wayClear):
                FindObjWithColor(color)
                return
        ColoredObjDist = FindDist(color)
        if ColoredObjDist is None:
            FindObjWithColor(color)
            return
        if ColoredObjDist[0] == 0:
            turnAround(ColoredObjDist[0],True)
        else:
            if 360-ColoredObjDist[0]>180:
                turnAround(ColoredObjDist[0],False)
            else:
                turnAround(360-ColoredObjDist[0],True)
    
    print "I found the Object!"

def findObj():
    

    print "Please enter a color"
    color = raw_input()
    FindObjWithColor(color)


if __name__ == '__main__':
	rospy.init_node('option4', anonymous=True)
        findObj()


