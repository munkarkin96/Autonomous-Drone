from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import time
import math
import argparse
import socket
import re
import rospy

from dronekit import connect, Command, VehicleMode
from pymavlink import mavutil

import rospy 
from std_msgs.msg import Float32
import time
from sensor_msgs.msg import Range

import argparse  
import cv2
import numpy as np
import math

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range 

class listener (object):
  
 	
	def callback (self, data):
		self.angleMax = data.angle_max
		self.dataRanges = data.ranges
		self.rangeMax = data.range_max
		self.angleIncrement = data.angle_increment 
		
	def callbackSonar (self, sonarData):
		if sonarData.range < 16: 
			self.groundDistance = sonarData.range
			
	def callbackLeft (self, sonarLeft):
		if sonarLeft.range < 16: 
			self.sonarLeft = sonarLeft.range
	
	def callbackRight (self, sonarRight):
		if sonarRight.range < 16: 
			self.sonarRight = sonarRight.range
	
		
	def __init__(self):
		
		rospy.init_node('indoorFlight', anonymous=True)
		
		rospy.Subscriber("/scan", LaserScan, self.callback)
		rospy.Subscriber("sonar_down", Range, self.callbackSonar)
		rospy.Subscriber("sonar_left", Range, self.callbackLeft)
		rospy.Subscriber("sonar_right", Range, self.callbackRight)
		
		self.angleMax = 0 
		self.dataRanges = 0
		self.rangeMax = 0
		self.angleIncrement = 0
		
		self.groundDistance = 0 
		self.sonarLeft = 0
		self.sonarRight = 0 
		
		self.f = 0
				
		
		self.rate = rospy.Rate(100)
	
		while True:
			
			def armingDrone():
				
				vehicle.parameters['ARMING_CHECK']=0

				time.sleep(0.1)
				print "Arming check status:  %s" % vehicle.parameters['ARMING_CHECK']

				print "Arming motors"
				vehicle.mode    = VehicleMode("STABILIZE")
				vehicle.armed   = True
				vehicle.flush()

				while not vehicle.armed:
					#vehicle.armed   = True
					print " Waiting for arming..."
					time.sleep(1)

				if vehicle.armed == True:
					print "Copter is armed"
				
	
			def modeChange(mode): 
				print "Changing the vehicle mode to %s" % mode
				vehicle.mode    = VehicleMode(mode)
				vehicle.flush()

				while (vehicle.mode != mode):
					print "Changing mode ......"
					vehicle.mode    = VehicleMode(mode)
					time.sleep(1)
					
			def printVelocity():
				
				#print "Velocity in Vx Vy Vz %s" %vehicle.velocity 
				vehicleVelocity = str(vehicle.velocity)
				vehicleVelocity = re.findall(r"-?\d.\d+", vehicleVelocity)
				Vx = float (vehicleVelocity[0])
				Vy = float (vehicleVelocity[1])
				Vz = float (vehicleVelocity[2])
				Vtotal = math.sqrt(Vx*Vx+Vy*Vy+Vz*Vz)
				print "Total velocity: %.3f" % Vtotal +" m/s"
				
		
		
			def takeOff(aTargetAltitude):
				print "Taking off!"
				#vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
				# Check that vehicle has reached takeoff altitude
				throttle = 1300 
	
	
				while True:
					
					distance = [] 
					
					
					for i in range(2):
						distance.append(0)
						print "Value of i: ", i 
						distance[i] = self.groundDistance 
						
						print "Distance[i]" , distance[i]
						print "Distance[i-1]" , distance[i-1]
						print "New throttle value: @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@", throttle 
						
						if distance[i] < distance[i-1]:
							throttle+= 5 
							
						
						vehicle.channels.overrides['3'] = throttle
						print "Frequency: ", self.f 
						printVelocity()
						print " Altitude: ", vehicle.location.global_relative_frame.alt 
						#Break and return from function just below target altitude.  
						print "Ground distance: ", self.groundDistance
						self.f += 1

						if self.groundDistance >=aTargetAltitude*0.95:
							print "Reached target altitude"
							#modeChange("BRAKE")
							self.f=0
														
							return throttle
							
						self.rate.sleep()
							
					#break
							
							
				
					
			parser = argparse.ArgumentParser()
			parser.add_argument('--connect', default='127.0.0.1:14550')
			args = parser.parse_args()
			
			
			while True: 
				try:
					problemStatus = 0
					print 'Connecting to vehicle on: %s' % args.connect
					vehicle = connect(args.connect, baud=57600, wait_ready=True)
				except socket.error:
					print "No server exists."
					problemStatus = 1
				except exceptions.OSError as e:
					print "No serial exists."
					problemStatus = 1
				except dronekit.APIException:
					print "Timeout!"
					problemStatus = 1
				except :
					print "Other error."
					problemStatus = 1

				if (problemStatus == 0):
					break 
					
			vehicle.parameters['THR_DZ'] = 100
			vehicle.parameters['PILOT_VELZ_MAX'] = 0 
			vehicle.parameters['SIM_GPS_DISABLE'] = 1
			#0 to enable GPS
			#PILOT_VELZ_MAX was by default 250 --> 2.5m/s
	
			while True: 
				
				armingDrone()
				
				aTargetAltitude = 2
				throttle = takeOff(aTargetAltitude)
				
				modeChange("ALT_HOLD")
				
				print "vehicle.channels['1']" ,vehicle.channels['1']
				print "Type of it: ", type(vehicle.channels['1'])
				
				initialRoll = 1500 
				
				#initialRoll = vehicle.channels['1'] 
				
				#print "initialRoll:", initialRoll
				#print "Type of initialRoll: ", type(initialRoll)
				
				time.sleep(5)
				
				while True:
					print "Ground distance: ", self.groundDistance 
					
					if self.sonarLeft < self.sonarRight:
						initialRoll = 1490
					if self.sonarLeft > self.sonarRight:
						initialRoll = 1510
						
					
					vehicle.channels.overrides['1'] = initialRoll 
					
					self.rate.sleep()
					
						
				
				
				modeChange("LAND")
				
				vehicle.close()
				
				break
				
			break
			
				
if __name__ == '__main__':
	listener()
	print "Closing" 

