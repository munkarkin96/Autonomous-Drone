#!/usr/bin/env python
from dronekit import connect
import time #for time.time() 

import rospy 
from sensor_msgs.msg import Range #for terarangerone sensor 
from std_msgs.msg import String #for obstacleStatus 
from std_msgs.msg import Float32 #for latitude & longitude 

import vehicle_config
from mavlink_messages import mavlinkMessages 
from function_bank import functionBank 
# TODO: camera callibration 

class listener(object):
	
	def __init__(self):	
		self.vehicle = self.connect_to_dronekit()
		rospy.init_node('mainCode', anonymous = True)
		self.start_time = time.time()	
		self.start_publish = time.time()
		
		self.rate_400 = 400
		self.rate = rospy.Rate(self.rate_400)  
		# my downward facing sensor 
		rospy.Subscriber('terarangerone4', Range, self.callback_tera)	
		rospy.Subscriber('obstacleStatus', String, self.callback_obstacle)
		rospy.Subscriber('latitude', Float32, self.callback_latitude)
		rospy.Subscriber('longitude', Float32, self.callback_longitude)	
		
		# TODO: publish current_vel to terarangerListener.py 
		# safety radius should be based on current_vel, and not a constant 
		
		# takeoff altitude 
		self.aTargetAltitude = 2     
		self.goto_targetAltitude = self.aTargetAltitude
		self.ground_distance = 0 
		self.obstacle = ''
		self.latitude = 0
		self.longitude = 0
		
		# create class instances from my custom libraries 
		self.mavlink_messages_bank = mavlinkMessages(self.vehicle)
		self.operational_function_bank = functionBank (self.vehicle)
		
		# to be deleted soon once this is setup in the pixhawk 
		# run it once in the pixhawk to change the parameters, not done yet 
		# this may be the problem that caused 'restarting RTL, terrain data missing ...' 
		self.vehicle.parameters['TERRAIN_ENABLE'] =0
		self.vehicle.parameters['TERRAIN_FOLLOW'] =0
		
		self.vehicle.groundspeed = 3 
		
		'''
		FIX ME not sure how to publish opencv images to ROS yet 
		might need a cv_bridge to convert between opencv image to ROS image and vice versa 
		
		'''
		
		
	def callback_tera(self, tera_data):
		self.ground_distance = tera_data.range
		self.start_publish = time.time() 
		# starts publishing after 10 seconds, apparently publishing immediately gives errors 
		# due to self.vehicle.connect is not connected yet 
		if (self.start_publish - self.start_time > 10):
			self.mavlink_messages_bank.distance_sensor(self.ground_distance*100)
		
		
	def callback_obstacle(self, obstacle_status):
		self.obstacle = obstacle_status.data
		
	def callback_latitude (self, latitude):
		# sometimes it will still give error saying can't publish float values although I've changed it from string to float
		# QUICKFIX set respawn = 'True' in the roslaunch file 
		self.latitude = latitude.data
				
	def callback_longitude (self, longitude):
		self.longitude = longitude.data 
		
		

	def connect_to_dronekit(self):
		
		'''
		------------------------ from vehicle_config.py --------------------------------
		'''		
		# no need to create a class instance for this library as I've done that in the library itself 
		connection_str = vehicle_config.config.get_string('dronekit', 'connection_string', '/dev/ttyACM1')
		connection_baud = vehicle_config.config.get_integer('dronekit', 'baud', 57600)
		print "Connecting to vehicle on %s, baud=%d" % (connection_str, connection_baud)
		return connect(connection_str, baud=connection_baud, wait_ready=True)
		
	
	def obstacle_avoidance(self):
		# TODO: need to check if this behaves as expected
		# changed some syntax 
		if self.obstacle!='NONE':
			print "Value of self.obstacleStatus: %s"%self.obstacle
			brake_status = 0
			while True:
				if brake_status == 0:
					self.operational_function_bank.mode_change("BRAKE")
					time.sleep(2)
					brake_status +=1
					self.operational_function_bank.mode_change("GUIDED")
				print "Obstacle detected, flying upwards"
				
				# FIXME implementation of PID 
				self.mavlink_messages_bank.send_ned_velocity(0,0,-0.2,1)
				
				print "It has rised till: %s" % self.vehicle.location.global_relative_frame.alt
				time.sleep(0.1)
				if self.obstacle == "NONE":
					print "Obstacle cleared"
					print "New altitude: %s" % self.vehicle.location.global_relative_frame.alt
					self.mavlink_messages_bank.send_ned_velocity(0,0,-0.2,1)
					#self.vehicle.simple_goto(target_location)
					break; 
					
	def goto_location(self, is_home = False):
		# TODO: changed some syntax, need to recheck 
		return_target_alt = 5 
		self.goto_current_time = time.time()
		while self.vehicle.mode.name == 'GUIDED':
			
			# in case the heartbeat gets too low that it becomes disconnected, reconnect immediately 
			if self.vehicle is None:
				print 'Connection to vehicle has lost, reconnecting ...' 
				self.connect_to_dronekit() 
			
			if self.operational_function_bank.check_home():
				if is_home:
					# only send the mavlink message once every two seconds 
					if (time.time() - self.goto_current_time > 2):
						print 'On the way home ...'
						target_reached = self.operational_function_bank.goto(self.vehicle.home_location.lat, self.vehicle.home_location.lon, return_target_alt)		
						self.goto_current_time = time.time()
						print 'Target reached status: ', target_reached 
				else:
					if (time.time() - self.goto_current_time > 2):
						print 'On the way to teddy bear ...'
						print 'Latitude of teddy bear: ', self.latitude
						print 'Longitude of teddy bear: ', self.longitude 
						target_reached = self.operational_function_bank.goto(self.latitude, self.longitude, self.goto_targetAltitude)
						self.goto_current_time = time.time()
						print 'Target reached status: ', target_reached 
				self.obstacle_avoidance()			
				if not target_reached is None:
					print 'Reached the target location.'
					break
			# but the frequency is set to be 400 Hz to detect obstacle quickly 
			# comment out self.obstacle_avoidance() if testing without the function is required 
			# reduce to frequency if self.obstacle_avoidance() is not required to save on processing power 
			self.rate.sleep()

		
	def main(self):
		
		'''
		*All the mavlink messages are found in mavlink_messages.py 
		-------------------- to send mavlink messages ----------------------
		self.mavlink_messages_bank.send_ned_velocity(0,0,0,0)    ---> example 
		
		
		*All operational functions are found in function_bank.py 
		------------------ to retrieve operational functions ---------------
		self.operational_function_bank.arming_drone()            ---> example 
			
		'''
				
		# to make sure that a location is acquired and home location is retrieved before proceeding 
		while True:
			if self.operational_function_bank.check_home():
				break
			else:
				time.sleep(1)
		# arming the drone with extensive error checking mechanism 
		self.operational_function_bank.arming_drone()
		# taking off while checking for the change in altitude 
		self.operational_function_bank.take_off(self.aTargetAltitude)
		# going to the teddy bear location, and follow the teddy bear if required with an update rate of 0.5Hz of the location 
		self.goto_location()
		# return to home position with a higher altitude to prepare for precision landing 
		# change return_target_alt if required 
		self.goto_location(is_home = True)
		# important to prevent the drone from flipping over after landing due to the inability to detect the change in altitude 
		self.operational_function_bank.land_disarm_close_vehicle()
				
	
vehicle_start = listener()
vehicle_start.main()
