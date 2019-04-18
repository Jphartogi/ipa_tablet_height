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

ter=0
tc=0.01

#gpio settings
wpi.wiringPiSetup()

class Node():
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
		self.TPM = float(rospy.get_param("~tick_per_meter","4342.2"))
		self.MAX_SPEED = float(rospy.get_param("~max_speed", "2.0"))
		self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.315"))
		self.rc = Roboclaw("/dev/roboclaw",115200)
		self.address = 0x80
		self.rc.Open()
		self.last_enc_value = 0
		self.last_enc_time = rospy.Time.now()
		self.total_dist = 0
		self.total_ticks = 0
		self.curr_dist = 0.0
		self.sub = rospy.Subscriber("/joy", Joy, self.callback_joy)
		self.service = rospy.Service('tabletheight', roboclaw_service, self.set_height_srvs)
		self.height_initialized = False
		self.speed = 0
		self.go_up = False
		self.go_down = False
		self.goal_reached = False
		self.initialization_mode = False
		self.find_height_mode = False
		self.height_confirmed = False
		self.encoder_value = 0
		
	
		# self.rc.SetPinFunctions(self.address, 0,2,0)
		
        

	def callback_joy(self,joy): # using joy buttons to move
		# speed = self.rc.ReadISpeedM1(self.address)
		self.calculate_encoder()
		if joy.buttons[2] == 1 or self.go_up == True:
			
			# in auto mode
			if self.find_height_mode == True:
				#set the emergency stop to be off so it can go up again.
				self.rc.SetPinFunctions(self.address, 0,0,0)
				if not self.dcc:  # acceleration mode
					if self.speed > 19:
						const_speed = self.speed
						self.forward(const_speed)
						# print "const speednya ",const_speed
					else:
						self.speed = self.speed + 1
						self.forward(self.speed)
						time.sleep(tc)
						# print "acc speednya ",self.speed
				else:   # decceleration mode
					if self.speed > 7:
						self.speed = self.speed - 1
						self.forward(self.speed)
						# print " dcc speed ",self.speed
					else:
						# print " dcc speed ",self.speed
						self.forward(self.speed)
						time.sleep(tc)

			else:   # for initialization and controlled with joystick
				self.rc.SetPinFunctions(self.address, 0,0,0)
				self.forward(20)
				print "++"
				
			# print "Pin Functions Value ",self.rc.ReadPinFunctions(self.address)
			# print "Read Error Value ",self.rc.ReadError(self.address)


		elif joy.buttons[1] == 1 or self.go_down == True:
			
			# in auto mode
			if self.find_height_mode == True:
				self.rc.SetPinFunctions(self.address, 0,2,0)
				if not self.dcc: # acceleration mode
					if self.speed > 19:
						const_speed = self.speed
						self.backward(const_speed)
						# print "const speednya ",const_speed
					else:
						self.speed = self.speed + 1
						# print "acc speednya ",self.speed
						self.backward(self.speed)
						time.sleep(tc)
				else: # decceleration mode
					if self.speed > 7:
						self.speed = self.speed - 1
						self.backward(self.speed)
						# print " dcc self.speed ",self.speed
					else:
						self.backward(self.speed)
						# print " dcc self.speed ",self.speed
						time.sleep(tc)
			else:  # for initialization and controlled with joystick
				self.rc.SetPinFunctions(self.address, 0,2,0)
				self.backward(20)
				print "--"
				
			
			# print "Pin Functions Value ",self.rc.ReadPinFunctions(self.address)
			# print "Read Error Value ",self.rc.ReadError(self.address)

		else:
			self.stop()

	def speed_control(self,curr_dist,goal_dist):
		total_distance = abs(goal_dist - self.curr_dist)
		distance_to_acc = total_distance / 5
		tolerance = 0.03
		# if the goal is higher than current pos
		if goal_dist > self.curr_dist:
			# when to deccelerate
			if curr_dist > goal_dist - distance_to_acc and curr_dist < goal_dist - tolerance :
				self.dcc = True
				self.acc = False
				
				#print " goal > curr . curr_dist, total_dist, dist_to_acc ,goal_dist ", curr_dist,total_distance,distance_to_acc,goal_dist
			# when to accelerate
			elif curr_dist < goal_dist - distance_to_acc and not self.goal_reached:
				self.dcc = False
				self.acc = True
				
				#print " goal > curr . curr_dist, total_dist, dist_to_acc ,goal_dist", curr_dist,total_distance,distance_to_acc,goal_dist
			# # when to constant
			else:
				self.constant = True
				self.acc = False
				self.dcc = False
				self.goal_reached = True
				
				#print " goal > curr . curr_dist, total_dist, dist_to_acc ,goal_dist", curr_dist,total_distance,distance_to_acc,goal_dist
				
		# if the goal is lower than current pos		
		else:
			# when to deccelerate
			if curr_dist < goal_dist + distance_to_acc and curr_dist > goal_dist + tolerance:
				self.dcc = True
				self.acc = False
				
				#print " goal > curr . curr_dist, total_dist, dist_to_acc ,goal_dist ", curr_dist,total_distance,distance_to_acc,goal_dist
			# when to accelerate
			elif curr_dist > goal_dist + distance_to_acc and not self.goal_reached:
				self.dcc = False
				self.acc = True
				
				#print " goal > curr . curr_dist, total_dist, dist_to_acc ,goal_dist", curr_dist,total_distance,distance_to_acc,goal_dist
			# # when to constant
			else:
				self.constant = True
				self.acc = False
				self.dcc = False
				self.goal_reached = True
				
				#print " goal > curr . curr_dist, total_dist, dist_to_acc ,goal_dist", curr_dist,total_distance,distance_to_acc,goal_dist

	def forward(self,speed):
		self.rc.ForwardM1(self.address,speed)
	def backward(self,speed):
		self.rc.BackwardM1(self.address,speed)
	def stop(self):
		self.rc.BackwardM1(self.address,0)
		self.rc.ForwardM1(self.address,0)
		self.go_down = False
		self.go_up = False
		self.speed = 0
	def dummy_init(self) :
		self.initialization_mode = True
		print "start initialization"
		# self.calculate_encoder()

		while self.height_initialized == False:
			if self.height_initialized == True:
				print "initialization complete"
				break
	
	def calculate_encoder(self):
		enc = self.rc.ReadEncM1(self.address)
		self.encoder_value = enc[1]
		########## in initialization mode  ###################
		if self.initialization_mode == True:
			self.go_down = True
			# set the s4 modes so it become an emergency stop
			self.rc.SetPinFunctions(self.address, 0,2,0)
			status = self.rc.ReadError(self.address)[1]
			print ("status value",status)
			# status of the emergency stop will be more than 0 if pressed
			if status > 0:
				self.go_down = False
				self.stop()
				self.height_initialized = True
				self.initialization_mode = False
				self.reset()
				

		########## in finding tablet height mode ###############
		if self.find_height_mode == True:
			
			self.go_up = True
			tolerance = 0.03
			distance = self.update(self.encoder_value)
			
			self.speed_control(distance,self.tablet_height)
			# if distance bigger than lower boundary and smaller than upper boundary
			if distance > self.tablet_height - tolerance and distance < self.tablet_height + tolerance:
				self.go_up = False
				self.stop()
				self.goal_reached = True
				self.curr_dist = distance
				
			elif distance < self.tablet_height - tolerance:
				self.go_up = True
				self.go_down = False
				self.goal_reached = False
			elif distance > self.tablet_height + tolerance:
				self.go_down = True
				self.go_up = False
				self.goal_reached = False

		return self.encoder_value


	def reset(self) :
		self.rc.ResetEncoders(self.address)


	def set_height_srvs(self,req):
		
		# get desired height [m] from service request
		self.tablet_height=req.enc
		if self.tablet_height > 80 or self.tablet_height < 0:
			rospy.loginfo("Tablet height is not possible! Please choose another tablet height")
			self.height_confirmed = False
			return roboclaw_serviceResponse(0)
				
		else:
			rospy.loginfo("Height is possible, proceeding")
			self.height_confirmed = True

		self.tablet_height = int(round(self.tablet_height/5))
		# enter value in cm
		print "Got Service, go to desired tablet height" , self.tablet_height

		# transfer height [m] into ticks
		goal_ticks = self.tablet_height
		print " detected goal is : ",goal_ticks
		
		# move to init position?	
		if goal_ticks == 0 and not self.height_initialized:
			self.dummy_init()

		else:
			# read encoder value
			if self.height_initialized:
				self.execute_tablet_height(goal_ticks)
			if not self.height_initialized:
				print "the height is not initialized, now initializing"
				self.dummy_init()
				if self.height_initialized:
					self.execute_tablet_height(goal_ticks)
				else:
					return
		
		
		self.height_confirmed = False
		return roboclaw_serviceResponse(goal_ticks)
		


	def execute_tablet_height(self,height):
		print "height accepted : ",height
		if self.height_confirmed:
			self.find_height_mode = True
			self.goal_reached = False
			
	
	def update(self, enc_value):
		ticks = enc_value - self.last_enc_value
		self.last_enc_value = enc_value
		self.total_ticks = self.total_ticks + ticks
		dist = self.total_ticks / self.TPM
		current_time = rospy.Time.now()
		d_time = (current_time - self.last_enc_time).to_sec()
		self.last_enc_time = current_time
        
		return dist

	
		
				
if __name__ == "__main__":
	rospy.init_node('ipa_tablet_height')
	node = Node()
	rospy.loginfo("tablet height control is running")
	rospy.spin()

