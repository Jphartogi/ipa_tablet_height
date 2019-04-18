#!/usr/bin/env python

NAME = 'roboclaw_node'

import time
from roboclaw_driver.roboclaw import Roboclaw
import serial
import rospy
import std_msgs.msg 
from std_msgs.msg import String
import wiringpi as wpi
from sensor_msgs.msg import Joy
from std_srvs.srv import Empty
from emasin_proxy.srv import *

import diagnostic_msgs
import diagnostic_updater

rc = Roboclaw("/dev/ttyACM0",115200)
address = 0x80
rc.Open()

ter=0
tc=0.01

#gpio settings
wpi.wiringPiSetup()

rc.SetPinFunctions(address, 0,2, 0)




def init():
	print "start init"
	while rc.ReadPinFunctions(address)[0] != -1:		# while end stop is not pressed
		rc.BackwardM1(address,15)
		print rc.ReadPinFunctions(address)
		print "encoder", rc.ReadEncM1(address)[1]

 
	reset() 
#	rc.BackwardM1(address,0)
	enc = rc.ReadEncM1(address)
	print "encoder val ", enc[1]
        print "init successful"


def reset() :
	rc.ResetEncoders(address) 

def forward(var):
	rc.ForwardM1(address,var)
	

def backward(var):
	rc.BackwardM1(address,var)

def stop():
	print "stop motor"
	rc.ForwardM1(address, 0)

def read_encoder():
	enc1 = rc.ReadEncM1(address)
	return enc1
	
def accer():
	print "switch to acceleration"
	global acc_ticks
	enc=read_encoder()
	count = 6
	acc_ticks=int(round(0.25 * dz))
	goal_acc = goal_ticks - dz + acc_ticks
	print "	acc steps = ", acc_ticks 

	if acc_ticks > 0: 
		while enc[1] <= goal_acc:

			enc=read_encoder()
			print "--", enc[1]

			backward(count)

			if count < 15:
				count=count + 2
			time.sleep(tc)

	if acc_ticks < 0: 
		while enc[1] >= goal_acc:

			enc=read_encoder()
			print  "++", enc[1]

			forward(count)

			if count < 15:
				count=count + 2
			time.sleep(tc)
	return count
		
		
def const(count):
	print "switch to constant"
	if count > 0:
		while rc.ReadEncM1(address)[1] <= 5000:
			print  "encoder value", rc.ReadEncM1(address)[1] 
			rc.ForwardM1(address,15)
			time.sleep(tc)

#	if count < 0:
#		while rc.ReadEncM1(address)[1] >= goal_const:
#			enc=read_encoder()
#			print  "		encoder value", enc[1]
#			rc.BackwardM1(address,15)
#			time.sleep(tc)
	
	return count
		


def dcc(count):
	print "switch to deceleration",
	global dcc_ticks
	enc = rc.ReadEncM1(address)
	dcc_ticks=int(round(0.25 * dz))
	goal_dcc = goal_ticks
	print "	dcc steps = ", dcc_ticks 

	if dcc_ticks > 0: 
		while enc[1] <= goal_dcc:

			enc=read_encoder()
			print "		encoder value", enc[1]

			backward(count)
			if count > 6:
				count=count - 1
			time.sleep(tc)

	if dcc_ticks < 0: 
		while enc[1] >= goal_dcc:
			enc=read_encoder()
			print "		encoder value", enc[1]

			forward(count)
			if count > 5:
				count=count - 1
			time.sleep(tc)
		

	
#def shut_down():	

	# if we get goal_ticks 0, then move very soft to endstop

def main_function(req):
	count = 0
	# get desired height [m] from service request
	tablet_height=req.enc 

	print "Got Service, go to desired tablet height" , tablet_height	
	

	# transfer height [m] into ticks 
	goal_ticks = tablet_height

	# move to init position?
	if goal_ticks == 0:
		init()
		#rc.ResetEncoders(address) 

	else:
		
		# read encoder value
		enc = rc.ReadEncM1(address)
		print "we are at [ticks]", enc[1]
	       	print "			and want to move to tablet_height [m]", tablet_height

		# calc the way we have to move
		dz = goal_ticks - enc[1]
		print "move [ticks]", dz
		result = const(dz)
		print result
#
#		#check if goal_ticks is out of range, if outside, then move to boarder
#		
#		count = accer()
#		count2 = const(count)
#		dcc(count2)
#		stop()

	
	return roboclaw_serviceResponse(goal_ticks)

	
def callback_joy(joy): # using joy buttons to move
	enc = rc.ReadEncM1(address)
	#print rc.ReadPinFunctions(address)
	#print "		encoder value", enc[1]
	speed = rc.ReadISpeedM1(address)
	#print speed[1]
	if joy.buttons[2] == 1:
		rc.ForwardM1(address,25)
		print "++"
		print "apaan sih valuenya ",rc.ReadPinFunctions(address)
		print "apaan sih valuenya ",rc.ReadError(address)

	elif joy.buttons[1] == 1:
		rc.BackwardM1(address,25)
		print "--"
		print "apaan sih valuenya ",rc.ReadPinFunctions(address)
		print "apaan sih valuenya ",rc.ReadError(address)

	else:
		rc.BackwardM1(address,0)


def service():
    rospy.init_node(NAME)
    #initiating service named ' roboclaw_servicere
    s = rospy.Service('tabletheight', roboclaw_service, main_function)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

class Node:
    def __init__(self):

        self.ERRORS = {0x0000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "Normal"),
                       0x0001: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (diagnostic_msgs.msg.DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (diagnostic_msgs.msg.DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M1 home"),
                       0x8000: (diagnostic_msgs.msg.DiagnosticStatus.OK, "M2 home")}


if __name__ == "__main__":

	#node = Node()
	rospy.Subscriber("/joy", Joy, callback_joy)
	service()

	

