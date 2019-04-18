#!/usr/bin/env python

import time
from roboclaw_driver.roboclaw import Roboclaw
import serial
import rospy
import std_msgs.msg 
from std_msgs.msg import String
import wiringpi as wpi
from sensor_msgs.msg import Joy


rc = Roboclaw("/dev/ttyACM0",115200)
address = 0x80

rc.Open()
global vel
vel = 0
vel_b = 0
wpi.wiringPiSetup()

wpi.pinMode(27, 0)
rc.SetPinFunctions(address, 0,2, 0)


def callback_joy(joy): # using joy buttons to move
	global vel
	global vel_b
	enc = rc.ReadEncM1(address)
	print rc.ReadPinFunctions(address)
	print "		encoder value", enc[1]
		
	if joy.buttons[2] == 1:
 		if vel <25:
	 		vel = vel +2
			print vel
			rc.ForwardM1(address,vel)
	

	elif joy.buttons[1] == 1:
		if vel_b <25:
	 		vel_b = vel_b +2
			print vel_b
			rc.BackwardM1(address,vel_b)

	else:
		#rc.BackwardM1(address,0)
		rc.ForwardM1(address,0)
 		if vel >0:
			vel = vel -2
			rc.ForwardM1(address,vel)
		if vel_b >0:
			vel_b = vel_b -2
			rc.BackwardM1(address,vel_b)




def listener():
    rospy.init_node('listener_node', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback_joy)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':

    listener()


