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

import termios

import serial
import os

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
		# get parameter
		self.TPM = float(rospy.get_param("~tick_per_meter","4342.2"))
		self.BASE_WIDTH = float(rospy.get_param("~base_width", "0.315"))
		self.max_speed = int(rospy.get_param("~max_speed","20"))
		self.min_speed = int(rospy.get_param("~min_speed","10"))
		self.baud = int(rospy.get_param("~baud","115200"))
		self.tolerance = float(rospy.get_param("~height_tolerance","0.1"))
		# initialization port and address
		self.rc = Roboclaw("/dev/roboclaw",self.baud)
		self.port = "/dev/roboclaw"
		self.address = 0x80
		self.rc.Open()
		
		self.last_enc_value = 0
		self.last_enc_time = rospy.Time.now()
		self.total_dist = 0
		self.total_ticks = 0
		self.last_goal = 0.0
		self.real_dist = 0.0
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
		self.trigger = False
		self.encoder_value = 0
		self.joy_pressed = False
		self.error_count = 0
		self.rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown(): # to loop over and over again
			self.callback()
			self.rate.sleep()


	def callback(self):
		# constantly calculate the value of the encoder!
		
		self.error_count = self.error_count + 1
		
		## to remove error caused by the bumper
		if self.error_count > 30:
			rospy.signal_shutdown("restarted to remove error!")		
		print "value error_count ", self.error_count
		
		# it will stuck here if there is an error
		##

		print "error value",self.rc.ReadError(self.address)
		print "read encoder value", self.rc.ReadEncM1(self.address)
		self.calculate_encoder()
	
		
		if self.go_up == True:
			
			# in auto mode
			if self.find_height_mode == True:
				
				# set the emergency stop to be off so it can go up again.
				self.rc.SetPinFunctions(self.address, 0,0,0)
				
				# acceleration mode
				if not self.dcc: 
					
					if self.speed > self.max_speed - 1: # if its at max speed, change to constant speed
						const_speed = self.speed
						self.forward(const_speed)
						
					else:
						
						self.speed = self.speed + 3
						self.forward(self.speed)
						time.sleep(tc)
				# decceleration mode		
				else:   
					
					if self.speed > self.min_speed - 1:
						# create a linear equation speed = x * d + c
						x = (self.max_speed - self.min_speed) / self.dcc_dist

						d = abs(self.real_dist - self.tablet_height)
						self.speed = int(round(x * d)) + (self.min_speed)
						# self.speed = self.speed - 1
			
						if self.speed > self.max_speed:
							self.forward(10)
						else:	
							self.forward(self.speed)
						
					else:
						self.forward(self.min_speed)
						time.sleep(tc)
		
			else:   # for initialization and controlled with joystick
				# set the find height mode to be false so it needs another service to go to another height
				self.rc.SetPinFunctions(self.address, 0,0,0)
				self.forward(20)
				print "++"

		elif self.go_down == True:
			
			# in auto mode
			if self.find_height_mode == True:
				
				# to turn on the emergency stop function for safety reason
				self.rc.SetPinFunctions(self.address, 0,2,0)
				# acceleration mode
				if not self.dcc: 
					
					if self.speed > self.max_speed - 1:
						const_speed = self.speed
						self.backward(const_speed)
						
					else:
						self.speed = self.speed + 3
						self.backward(self.speed)
						time.sleep(tc)
				else: # decceleration mode
					
					if self.speed > self.min_speed - 1:
						x = (self.max_speed - self.min_speed) / self.dcc_dist # determine a constant
						d = abs(self.real_dist - self.tablet_height)
						self.speed = int(round(x * d))+ self.min_speed
					
						# for protection
						if self.speed > self.max_speed:
							self.backward(self.min_speed)
						# self.speed = self.speed - 1
						else:
							self.backward(self.speed)
					else:
						self.backward(self.min_speed)
						# print " dcc self.speed ",self.speed
						time.sleep(tc)

			else:  # for initialization and controlled with joystick
				
				self.rc.SetPinFunctions(self.address, 0,2,0)
				self.backward(20)
				print "--"

		else:		
			
			self.stop()
		
		self.error_count = 0 # this means the program is executed and no error!	


			# print "Pin Functions Value ",self.rc.ReadPinFunctions(self.address)
			# print "Read Error Value ",self.rc.ReadError(self.address)



	def callback_joy(self,joy): # using joy buttons to move
		# speed = self.rc.ReadISpeedM1(self.address)
		
		
		# constantly calculate the value of the encoder!
		
		if joy.buttons[2] == 1 or joy.buttons[1] == 1:
			self.find_height_mode = False
			self.joy_pressed = True
		

		if joy.buttons[1] == 0 or joy.buttons[2] == 0:
			self.joy_pressed = False
			self.go_down = False
			self.go_up = False
			
			

		if joy.buttons[2] == 1:
			self.go_up = True
			self.go_down = False
			

		elif joy.buttons[1] == 1:
			
			self.go_down = True
			self.go_up = False
	

		# else:
		# 	self.stop()
		

	def calculate_encoder(self):
		enc = self.rc.ReadEncM1(self.address)
		self.encoder_value = enc[1]

		########## in initialization mode  ###################
		if self.initialization_mode == True:
			self.go_down = True
			# set the s4 modes so it become an emergency stop
			self.rc.SetPinFunctions(self.address, 0,2,0)
			status = self.rc.ReadError(self.address)[1]
			# print ("status value ",status)
			# status of the emergency stop will be more than 0 if pressed
			if status > 0:
				self.go_down = False
				self.stop()
				self.height_initialized = True
				self.initialization_mode = False
				self.reset()
				

		########## in finding tablet height mode ###############
		if self.find_height_mode == True and self.height_confirmed == True:
			
			self.go_up = True
			tolerance = self.tolerance
			distance = self.update(self.encoder_value)

			self.real_dist = distance
			self.speed_control(distance,self.tablet_height)

			# if distance bigger than lower boundary and smaller than upper boundary
			# send a command / status to go up or go down or to stop
			if distance > self.tablet_height - tolerance and distance < self.tablet_height + tolerance:
				
				self.go_up = False
				self.stop()
				self.goal_reached = True
				self.last_goal = self.real_dist
				
			elif distance < self.tablet_height - tolerance:
				self.go_up = True
				self.go_down = False
				self.goal_reached = False

			elif distance > self.tablet_height + tolerance:
				self.go_down = True
				self.go_up = False
				self.goal_reached = False


		else:
			pass

		return self.encoder_value
		

	def speed_control(self,curr_dist,goal_dist):
		total_distance = abs(goal_dist - self.last_goal)
		
		# as the total distance is increasing, the distance to decelerate also increasing
		distance_to_acc = total_distance / (total_distance + 4)
		tolerance = self.tolerance
		# if the goal is higher than current pos
		if goal_dist > self.last_goal:
			# when to deccelerate
			if curr_dist > goal_dist - distance_to_acc and curr_dist < goal_dist - tolerance :
				# calculate the remaining distance to target for the decceleration
				if not self.trigger:
					self.dcc_dist = abs(goal_dist - curr_dist) # take the distance only once!
					# set a trigger so it take only the first value
					self.trigger = True
				self.dcc = True
				self.acc = False
			
			# when to accelerate
			elif curr_dist < goal_dist - distance_to_acc and not self.goal_reached:
				self.dcc = False
				self.acc = True
				self.trigger = False
				
			# when to constant
			else:
				
				self.acc = False
				self.dcc = False
				self.goal_reached = True
				self.trigger = False
				
				
				
		# if the goal is lower than current pos		
		else:
			# when to deccelerate
			if curr_dist < goal_dist + distance_to_acc and curr_dist > goal_dist + tolerance:
				# calculate the remaining distance to target for the decceleration
				
				if not self.trigger:
					self.dcc_dist = abs(goal_dist - curr_dist) # take the distance only once!
					self.trigger = True
				self.dcc = True
				self.acc = False
				
			# when to accelerate
			elif curr_dist > goal_dist + distance_to_acc and not self.goal_reached:
				self.dcc = False
				self.acc = True
				self.trigger = False
				
			# # when to constant
			else:
				self.trigger = False
				self.acc = False
				self.dcc = False
				self.goal_reached = True
	
	def reset(self) :
		self.rc.ResetEncoders(self.address)
	
	def set_height_srvs(self,req):

		self.height_confirmed = False
		# get desired height [m] from service request
		self.tablet_height=req.enc

		# limit the height requested
		
		while self.tablet_height > 80 or self.tablet_height < 0:
			rospy.loginfo("Tablet height is not possible! Please choose another tablet height")
			self.height_confirmed = False
			return roboclaw_serviceResponse(0)
			
		else:
			rospy.loginfo("Height is possible, proceeding")
			self.height_confirmed = True

		if self.height_confirmed:

			# this calculation is done manually, by manual measurement 
			self.tablet_height = int(round(self.tablet_height/5))

			# enter value in cm
			print "Got Service, go to desired tablet height" , self.tablet_height*5

			# transfer height [m] into ticks
			goal_ticks = self.tablet_height
			
			
			# move to init position?	
			if goal_ticks == 0 and not self.height_initialized:
				self.init()

			else:
				
				if self.height_initialized:
					self.execute_tablet_height(goal_ticks)
				if not self.height_initialized:
					print "the height is not initialized, now initializing"
					self.init()
					if self.height_initialized:
						self.execute_tablet_height(goal_ticks)
					else:
						return
			
			
			return roboclaw_serviceResponse(goal_ticks*5)
		
		else:
			return roboclaw_serviceResponse(0)
				
		
	
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

	def init(self) :
		self.initialization_mode = True
		print "start initialization"
		# self.calculate_encoder()

		while self.height_initialized == False:
			if self.height_initialized == True:
				print "initialization complete"
				break
	
	def execute_tablet_height(self,height):
		
		if self.height_confirmed:
			self.find_height_mode = True  # trigger the status of automatic tablet height finding mode to true!
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
	rospy.init_node('ipa_tablet_height',disable_signals=True) # disable signals so it can be shutdown from the code!
	node = Node()
	rospy.loginfo("tablet height control is running")
	rospy.spin()











