#!/usr/bin/env python

## Simple myo demo that listens to std_msgs/UInt8 poses published 
## to the 'myo_gest' topic and drives turtlesim

import rospy, math
from std_msgs.msg import UInt8, String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Vector3
from ros_myo.msg import EmgArray, MyoPose

if __name__ == '__main__':
	global movingState
	global y
	global x
	global K
	global angular_speed

	angular_speed = 0
	movingState = 0
	y = 0
	x = 0
	rospy.init_node('turtlesim_driver', anonymous=True)

	# Publish to the turtlesim movement topic
	tsPub = rospy.Publisher("turtle1/cmd_vel", Twist, queue_size=1)
	turtlesimPub = rospy.Publisher("directs", String, queue_size=10)

	# Use the calibrated Myo gestures to drive the turtle
	def drive(gest):
		global movingState
		#global speed
		if gest.pose == 1: #REST
			movingState = 0
		elif gest.pose == 2: #FIST
			movingState = -0.2
		elif gest.pose == 5 : #FINGERS_SPREAD
			movingState = 0.2

		if movingState > 0 :
			turtlesimPub.publish("go forward")
		elif movingState < 0 :
			turtlesimPub.publish("go back")
		else:
			turtlesimPub.publish("stop")
		tsPub.publish(Twist(Vector3(movingState, 0, 0), Vector3(0, 0, 0)))


	def turn(imuRead):
		global y
		global x
		global K
		global angular_speed
		y = imuRead.linear_acceleration.y
		x = imuRead.linear_acceleration.x
		K = 2
		# read acceleration when arm is raised
		if x<0:
			if y>0:
				angular_speed = abs(x)
				tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,K*angular_speed)))
				turtlesimPub.publish("turn CW")
			
			if y<0:
				angular_speed = abs(x)
				tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-K*angular_speed)))
				turtlesimPub.publish("turn CCW")


	def strength(emgArr1):
		emgArr=emgArr1.data
		# Define proportional control constant:
		K = 0.005
		# Get the average muscle activation of the left, right, and all sides
		aveRight=(emgArr[0]+emgArr[1]+emgArr[2]+emgArr[3])/4
		aveLeft=(emgArr[4]+emgArr[5]+emgArr[6]+emgArr[7])/4
		ave=(aveLeft+aveRight)/2
		# If all muscles activated, drive forward exponentially
		if ave > 500:
			tsPub.publish(Twist(Vector3(0.1*math.exp(K*ave),0,0),Vector3(0,0,0)))
		# # If only left muscles activated, rotate proportionally
		# elif aveLeft > (aveRight + 200):
		# 	tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,K*ave)))
		# # If only right muscles activated, rotate proportionally
		# elif aveRight > (aveLeft + 200):
		# 	tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,-K*ave)))
		# # If not very activated, don't move (high-pass filter)
		# else:
		# 	tsPub.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))

	rospy.Subscriber("myo_imu", Imu, turn)
	#rospy.Subscriber("myo_gest", MyoPose, drive)
	rospy.Subscriber("myo_emg", EmgArray, strength)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
