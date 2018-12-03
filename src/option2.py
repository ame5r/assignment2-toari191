#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def rotate():

    rospy.init_node('rotating', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    VelocityMessage = Twist()

    speed = 10 
    angle = input("Type your degrees:")
    clockwise = True

    #Converting from degrees to radians
    angular_speed = speed*2*PI/360
    relative_angle = angle*2*PI/360

    VelocityMessage.linear.x=0
    VelocityMessage.linear.y=0
    VelocityMessage.linear.z=0
    VelocityMessage.angular.x = 0
    VelocityMessage.angular.y = 0
    if clockwise:
        VelocityMessage.angular.z = -abs(angular_speed)
    else:
        VelocityMessage.angular.z = abs(angular_speed)
    startNow = rospy.Time.now().to_sec()
    angleNow = 0
    while(angleNow < relative_angle):
        velocity_publisher.publish(VelocityMessage)
        timeNow = rospy.Time.now().to_sec()
        angleNow = angular_speed*(timeNow-startNow)
    #Stop robot now
    VelocityMessage.angular.z = 0
    velocity_publisher.publish(VelocityMessage)
    rospy.spin()
    rospy.sleep(0.5)
    return

if __name__ == '__main__':
    try:
     
        rotate()
    except rospy.ROSInterruptException:
        pass
