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


ranges= {'blue': ([80,0,0],[255,60,60]),
                  'red': ([0,0,80],[30,30,255]),
                  'green' : ([0,80,0],[80,255,60])}





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
    

def distance():
    print "Please enter a color"
    color = raw_input()
    FindDist(color)

if __name__ == '__main__':
	rospy.init_node('option3', anonymous=True)
        distance()

