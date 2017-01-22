#!/usr/bin/env python

from dronekit import VehicleMode #for mode_change() 
from dronekit import LocationGlobal, LocationGlobalRelative #for get_location_metres()
import time #for time.sleep() 
import math #for get_location_metres etc .. 

import rospy

import cv2
import numpy as np

from mavlink_messages import mavlinkMessages 
from position_vector import PositionVector
#from camera_callibration import cameraCallibration


class functionBank (object):
	
	def __init__ (self, dronekit_vehicle):
		self.vehicle = dronekit_vehicle 
		self.rate = rospy.Rate(400)
		self.rate_20 = rospy.Rate(20)
				
		self.mavlink_messages_bank = mavlinkMessages(self.vehicle)
		#self.camera_callibration_bank = cameraCallibration()
		
		self.home_initialised = False 
		self.disarmed_closed = False 
		self.mission_cmds = False 
		
		
	def take_off (self, aTargetAltitude):
		print "Taking off!"
		self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
		# Check that vehicle has reached takeoff altitude
		last_checked = time.time()
		while True:
			# checks for the altitude only every two seconds 
			if (time.time() - last_checked > 2):
				print 'Taking off, current altitude: ', self.vehicle.location.global_relative_frame.alt
				last_checked = time.time()
			if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 	
				print 'Altitude after takeoff: ', self.vehicle.location.global_relative_frame.alt				
				break
			# rated at high frequency to prevent overshooting 
			self.rate.sleep()
			
	def arming_drone(self):	
		# disable arming checks 
		self.vehicle.parameters['ARMING_CHECK']=0

		print "Arming motors"
		self.vehicle.mode    = VehicleMode("GUIDED")
		self.vehicle.armed   = True
		self.vehicle.flush()

		while not self.vehicle.armed:
			# arm it again in case the message wasn't received properly 
			# FIXME use COMMAND_ACK() to check if the message has receieved properly 
			self.vehicle.armed   = True
			print " Waiting for arming..."
			time.sleep(1)

		if self.vehicle.armed == True:
			print "Copter is armed"
	
	
	def mode_change(self, mode):
		# FIXME might need to craft a mavlink message to change to Guided_NoGPS 
		print "Changing the vehicle mode to %s" % mode
		self.vehicle.mode    = VehicleMode(mode)
		self.vehicle.flush()

		while (self.vehicle.mode != mode):
			print "Changing mode ......"
			self.vehicle.mode    = VehicleMode(mode)
			time.sleep(1)
		print "Mode has been changed to %s " %mode 
		
	def get_bearing(self, aLocation1, aLocation2):
		# checked 
		# gets the bearing and returns a data type float 
		off_x = aLocation2.lon - aLocation1.lon
		off_y = aLocation2.lat - aLocation1.lat
		bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
		if bearing < 0:
			bearing += 360.00
		return bearing;
		

	def get_distance_metres(self, aLocation1, aLocation2):
		# checked 
		# gets the distance between two location and return a data type float
		dlat = aLocation2.lat - aLocation1.lat
		dlong = aLocation2.lon - aLocation1.lon
		return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5
		
	def get_location_metres(self, original_location, dNorth, dEast):
				
		earth_radius = 6378137.0 #Radius of "spherical" earth
		#Coordinate offsets in radians
		dLat = dNorth/earth_radius
		dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

		#New position in decimal degrees
		newlat = original_location.lat + (dLat * 180/math.pi)
		newlon = original_location.lon + (dLon * 180/math.pi)
		if type(original_location) is LocationGlobal:
			targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
		elif type(original_location) is LocationGlobalRelative:
			targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
		else:
			raise Exception("Invalid Location object passed")
			
		return targetlocation;
				
		
	def goto_north_east(self, dNorth, dEast):
		# go to a specific location with given distance referenced from North, and East 
		current_location=self.vehicle.location.global_relative_frame
		target_location=self.get_location_metres(current_location, dNorth, dEast)
		target_distance=self.get_distance_metres(current_location, target_location)
		self.vehicle.simple_goto(target_location)

		while self.vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
			remaining_distance=self.get_distance_metres(self.vehicle.location.global_frame, target_location)
			print "Distance to target: ", remaining_distance
			if remaining_distance<=target_distance*0.05: #Just below target, in case of undershoot.
				print "Reached target"
				break;
			time.sleep(2)
			

	def goto(self, latitude, longitude,targetAltitude):		
		target_location = LocationGlobalRelative(latitude, longitude,targetAltitude)			
		current_location=self.vehicle.location.global_relative_frame
		target_distance=self.get_distance_metres(current_location, target_location)
		target_bearing=self.get_bearing(current_location, target_location) 
		# FIXME bearing has no purpose yet 
		print "Bearing: ", target_bearing #bearing returns float 
		self.vehicle.simple_goto(target_location)

		print "Current altitude: %s" % self.vehicle.location.global_relative_frame.alt
		remaining_distance=self.get_distance_metres(self.vehicle.location.global_frame, target_location)
		print "Distance to target: ", remaining_distance			
		
		if remaining_distance<=target_distance*0.05: #Just below target, in case of undershoot.
			print "Reached target"
			target_reached = 1
			return target_reached 
		
		# QUICKFIX sometimes target_distance*0.05 is too sensitive, the change in distance is not significant enough to break the loop 
		if remaining_distance < 0.5:
			print "Reached target"
			target_reached = 1
			return target_reached 

		self.rate.sleep() #400Hz 
		
	
		
	def precision_landing (self):
		# FIXME find a way to accurately measure the hsv values 
		hue_lower = 124
		hue_upper = 255
		saturation_lower = 110
		saturation_upper = 255
		value_lower = 70
		value_upper = 250
		min_contour_area = 500 # the smallest number of pixels in a contour before it will register this as a target
		

		#camera
		horizontal_fov = 170 * math.pi/180
		vertical_fov = 69.5 * math.pi/180
		horizontal_resolution = 1920
		vertical_resolution = 1080

		camera = cv2.VideoCapture(0)
		
		# FIXME this time.sleep() may not be necessary once cameraCallibration() object is done properly 
		# check if the camera has fully initialised instead 
		time.sleep(5) #for the camera to initialise 
		
		self.mode_change('LAND')
		
		while True:
			print '------------------------ runnng precision_landing ----------------------'
			_,capture = camera.read()
			hsvcapture = cv2.cvtColor(capture,cv2.COLOR_BGR2HSV)   
			inrangepixels = cv2.inRange(hsvcapture,np.array((hue_lower,saturation_lower,value_lower)),np.array((hue_upper,saturation_upper,value_upper)))#in opencv, HSV is 0-180,0-255,0-255
			tobecontourdetected = inrangepixels.copy()
			_,contours,hierarchy = cv2.findContours(tobecontourdetected,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
			
			contour_sizes=[]
			contour_centroids = []
			for contour in contours:  
				real_area = cv2.contourArea(contour)
				if real_area > min_contour_area:
					M = cv2.moments(contour) 
					cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
					cv2.circle(capture,(cx,cy),5,(0,0,255),-1)
					contour_sizes.append(real_area)
					contour_centroids.append((cx,cy))
			
			#find biggest contour (by area)    
			biggest_contour_index = 0
			for i in range(1,len(contour_sizes)):
				if contour_sizes[i] > contour_sizes[biggest_contour_index]:
					biggest_contour_index = i
			biggest_contour_centroid=None
			if len(contour_sizes)>0:
				biggest_contour_centroid=contour_centroids[biggest_contour_index]
			
			#if the biggest contour was found, color it blue and send the message
			if biggest_contour_centroid is not None:
				
				cv2.circle(capture,biggest_contour_centroid,5,(255,0,0),-1)
				x,y = biggest_contour_centroid
				print (x,y)
				self.mavlink_messages_bank.send_land_message(x, y, horizontal_resolution, horizontal_fov, vertical_resolution, vertical_fov)
			
			cv2.imshow('capture',capture) 
			cv2.imshow('inrangepixels',inrangepixels)
			
			
			threshold_alt_land = 5 
			print "Current altitude: ", self.vehicle.location.global_relative_frame.alt
			
			if self.vehicle.location.global_relative_frame.alt < threshold_alt_land:
				self.vehicle.parameters['LAND_SPEED'] = 15
				
			if self.vehicle.location.global_relative_frame.alt > threshold_alt_land:
				self.vehicle.parameters['LAND_SPEED'] = 50 
				
			if self.vehicle.location.global_relative_frame.alt < 0.5 : 
				time.sleep(0.1)
				while True:
					self.vehicle.armed = False 
					self.vehicle.flush()
					
					if self.vehicle.armed == False:
						print "Closing vehicle."
						self.vehicle.close() 
						self.disarm_closed = True 
						break
					# FIXME is this break required? 
					break
				break 
	
			if cv2.waitKey(1) == 'q' and 0xff ==ord('q'):
				break
			# low frequency to save on processing power, and pixhawk is good for 20Hz 
			self.rate_20.sleep()
				
		cv2.destroyAllWindows()
		camera.release()
		
	def fetch_mission(self):
		# do not have a mission yet, but is required to get my self.vehicle.home_location 
		# can be used in the future for auto mission if desired 
		print 'Fetching mission ...'
		self.mission_cmds = self.vehicle.commands
		self.mission_cmds.download()
		self.mission_cmds.wait_ready()
		if not self.mission_cmds is None:
			print "retrieved mission with %d commands" % self.mission_cmds.count
		else:
			print "failed to retrieve mission"
		
		
		
	def check_home(self):
		if self.home_initialised:
			return True 
			
		if self.vehicle.gps_0.fix_type < 2:
			print "Waiting for GPS ......." , self.vehicle.gps_0.fix_type
			return False 
		
		if self.vehicle.location.global_relative_frame is None:
			print 'Waiting for vehicle position ...'
			return False 
			
		if self.vehicle.location.global_relative_frame.lat is None or self.vehicle.location.global_relative_frame.lon is None or self.vehicle.location.global_relative_frame.alt is None:
			print 'Waiting for vehicle position ...'
			return False 
			
		if self.mission_cmds is None:
			self.fetch_mission()
			return False 
		
		if self.vehicle.home_location is None:
			print 'Waiting for home location ...'
			self.fetch_mission()
			return False 
			
		PositionVector.set_home_location(LocationGlobal(self.vehicle.home_location.lat,self.vehicle.home_location.lon,0))
		self.home_initialised = True 
		print 'Home location acquired' 
		
		return self.home_initialised 
		
			
	def land_disarm_close_vehicle(self):
		if self.disarmed_closed:
			return 	
		self.mode_change('LAND')
		while self.vehicle.location.global_relative_frame.alt > 0.5 :
			print 'Landing, current altitude: ', self.vehicle.location.global_relative_frame.alt
			time.sleep(2)
		print 'Landed'
		
		self.vehicle.armed = False 
		if not self.vehicle.armed:
			self.vehicle.close()
			self.disarmed_closed = True

		
		
			
			
			
	
		
'''
	def precision_landing_refined (self):
		
		self.camera_callibration_bank.init_camera()
		hue_lower = 124
		hue_upper = 255
		saturation_lower = 110
		saturation_upper = 255
		value_lower = 70
		value_upper = 250
		min_contour_area = 500 # the smallest number of pixels in a contour before it will register this as a target
		
		#camera
		horizontal_fov_radians = self.camera_callibration_bank.horizontal_fov *math.pi/180  #radians
		vertical_fov_radians = self.camera_callibration_bank.horizontal_fov *math.pi/180  #radians
		
		horizontal_resolution = self.camera_callibration_bank.img_width
		vertical_resolution = self.camera_callibration.img_height
		
		self.mode_change('LAND')
		
		while True:
			print '------------------------ runnng precision_landing ----------------------'
			_,capture = self.camera_callibration_bank.camera.read()
			hsvcapture = cv2.cvtColor(capture,cv2.COLOR_BGR2HSV)   
			inrangepixels = cv2.inRange(hsvcapture,np.array((hue_lower,saturation_lower,value_lower)),np.array((hue_upper,saturation_upper,value_upper)))#in opencv, HSV is 0-180,0-255,0-255
			tobecontourdetected = inrangepixels.copy()
			#TODO filter better. binary morphology would be a good start.
			_,contours,hierarchy = cv2.findContours(tobecontourdetected,cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
			
			contour_sizes=[]
			contour_centroids = []
			for contour in contours:  
				real_area = cv2.contourArea(contour)
				if real_area > min_contour_area:
					M = cv2.moments(contour) #moment is centroid
					cx,cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
					cv2.circle(capture,(cx,cy),5,(0,0,255),-1)
					contour_sizes.append(real_area)
					contour_centroids.append((cx,cy))
			
			#find biggest contour (by area)    
			biggest_contour_index = 0
			for i in range(1,len(contour_sizes)):
				if contour_sizes[i] > contour_sizes[biggest_contour_index]:
					biggest_contour_index = i
			biggest_contour_centroid=None
			if len(contour_sizes)>0:
				biggest_contour_centroid=contour_centroids[biggest_contour_index]
			
			#if the biggest contour was found, color it blue and send the message
			if biggest_contour_centroid is not None:
				
				cv2.circle(capture,biggest_contour_centroid,5,(255,0,0),-1)
				x,y = biggest_contour_centroid
				print (x,y)
				self.mavlink_messages_bank.send_land_message(x, y, horizontal_resolution, horizontal_fov, vertical_resolution, vertical_fov)
			
			cv2.imshow('capture',capture) 
			cv2.imshow('inrangepixels',inrangepixels)
			
			
			threshold_alt_land = 5 
			print "Current altitude: ", self.vehicle.location.global_relative_frame.alt
			
			if self.vehicle.location.global_relative_frame.alt < threshold_alt_land:
				self.vehicle.parameters['LAND_SPEED'] = 15
				
			if self.vehicle.location.global_relative_frame.alt > threshold_alt_land:
				self.vehicle.parameters['LAND_SPEED'] = 50 
				
			if self.vehicle.location.global_relative_frame.alt < 0.5 : 
				time.sleep(0.1)
				while True:
					self.vehicle.armed = False 
					self.vehicle.flush()
					
					if self.vehicle.armed == False:
						print "Closing vehicle."
						self.vehicle.close() 
						break
					break
				break 
			
					
				
			if cv2.waitKey(1) == 'q' and 0xff ==ord('q'):
				break

	
			
			self.rate_20.sleep()
				
		cv2.destroyAllWindows()
		self.camera_callibration_bank.camera.release()
				
'''

	
