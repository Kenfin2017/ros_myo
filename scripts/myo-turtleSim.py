#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic

import rospy
from std_msgs.msg import UInt8, String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Imu
from ros_myo.msg import MyoArm, MyoPose
from math import pi

########## Data Enums ###########
# MyoArm.arm___________________ #
#    UNKNOWN        = 0 	#
#    RIGHT          = 1		#
#    Left           = 2		#
# MyoArm.xdir___________________#
#    UNKNOWN        = 0		#
#    X_TOWARD_WRIST = 1		#
#    X_TOWARD_ELBOW = 2		#
# myo_gest UInt8________________#
#    REST           = 0		#
#    FIST           = 1		#
#    WAVE_IN        = 2		#
#    WAVE_OUT       = 3		#
#    FINGERS_SPREAD = 4		#
#    THUMB_TO_PINKY = 5		#
#    UNKNOWN        = 255	#
#################################


if __name__ == '__main__':

    global armState
    global xDirState    
    #global PI
    global linearVel
    global angularVel
    global moving
    global x
    global y
    global z
    
    x = 0
    y = 0
    z = 0
    angularVel = 0
    linearVel = 0
    #PI = 3.1415926535897
    armState = 1;

    rospy.init_node('turtlesim_driver', anonymous=True)

    turtlesimPub = rospy.Publisher("directs", String, queue_size=10)
    velPub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=10)

    # set the global arm states
    def setArm(data):
        global armState
        global xDirState

        armState = data.arm
        xDirState = data.xdir

    rospy.sleep(2.0)

    def orient(imuRead):
        # read orientation of myo 
        global z
        global x
        global PI
        z = imuRead.linear_acceleration.z
        y = imuRead.linear_acceleration.y
        x = imuRead.linear_acceleration.x
        # read acceleration when arm is raised
        if x<z:
            if y>0:
                angularVel = abs(y)*pi
                velPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,angularVel)))
                turtlesimPub.publish("turn CW")
            
            if y<0:
                angularVel = abs(y)*pi
                velPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-angularVel)))
                turtlesimPub.publish("turn CCW")
                
    # Use the calibrated Myo gestures to drive the turtle
    def drive(gest):
        global armState
        global xDirState
        global linearVel
        global angularVel
        #global PI
        global moving


        if gest.pose == 2: #FIST
            turtlesimPub.publish("go back")
            # if this pose is repeated keep increasing speed to 0.5 
            if abs(linearVel) < 0.5:
                linearVel -= 0.2
            angularVel = 0
            
        elif gest.pose == 3 and armState == 1 : #WAVE_IN, RIGHT arm
            turtlesimPub.publish("turn CCW")
            linearVel = 0
            angularVel = 0.2*pi
            
        elif gest.pose == 3 and armState == 2: #WAVE_IN, LEFT arm
            turtlesimPub.publish("turn CW")
            linearVel = 0
            angularVel = -0.2*pi
            
        elif gest.pose == 4 and armState == 1: #WAVE_OUT, RIGHT arm
            turtlesimPub.publish("turn CW")
            linearVel = 0
            angularVel = -0.2*pi
                     
        elif gest.pose == 4 and armState == 2: #WAVE_OUT, LEFT arm
            turtlesimPub.publish("turn CCW")
            linearVel = 0
            angularVel = 0.2*pi
            
        elif gest.pose == 5: #FINGERS_SPREAD
            turtlesimPub.publish("go forward")
            if abs(linearVel) < 0.5:
                linearVel += 0.2
            angularVel = 0
            
        elif gest.pose == 6: #THUMB TO PINKY
            turtlesimPub.publish("stop")
            linearVel = 0
            angularVel = 0
            
        else: #REST
            #linearVel = 0
            #angularVel = 0
            turtlesimPub.publish("none")

        velPub.publish(Twist(Vector3(linearVel, 0, 0), Vector3(0, 0, angularVel)))

    #rospy.Subscriber("myo_imu", Imu, orient)
    rospy.Subscriber("myo_gest", MyoPose, drive)
    rospy.loginfo('Please sync the Myo')

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
