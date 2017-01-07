import dronekit 
import socket
import exceptions
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import time
import math
import argparse
import re
import datetime 
import subprocess

import rospy 
from std_msgs.msg import String
from sensor_msgs.msg import Range
from px_comm.msg import OpticalFlow
from std_msgs.msg import Float32


class listener (object):
  
 	def callbackOptical (self,opticalData):
		#use the data from teraranger instead
		#self.groundDistance = opticalData.ground_distance
		self.flowX = opticalData.flow_x
		self.flowY = opticalData.flow_y
		self.Vx = opticalData.velocity_x
		self.Vy = opticalData.velocity_y

	def callback(self, obstacleStatus):
		self.obstacle = obstacleStatus.data
	
	def callbackLatitude (self, latitude):
		self.latitude = latitude.data
		
	def callbackLongitude (self, longitude):
		self.longitude = longitude.data 
		
	def callbackTera (self, teraData):
		self.groundDistance = teraData.range
		
	def __init__(self):
		
		rospy.init_node('mainCode', anonymous=True)
		
		rospy.Subscriber('obstacleStatus', String, self.callback)
		
		rospy.Subscriber('latitude', Float32, self.callbackLatitude)
		rospy.Subscriber('longitude', Float32, self.callbackLongitude)
		
		rospy.Subscriber('/px4flow/opt_flow', OpticalFlow, self.callbackOptical)
		
		rospy.Subscriber('terarangerone', Range, self.callbackTera)
		
		pubVelocity = rospy.Publisher('currentVel', Float32, queue_size = 0)

		
		self.obstacle = ''
		self.latitude = 0
		self.longitude = 0
		
		self.groundDistance = 0
		self.flowX = 0
		self.flowY = 0
		self.Vx = 0
		self.Vy = 0
		
		self.rate = rospy.Rate(500)
		self.frequency = 0 
		self.mavlinkCheck = '' 
		
		while True:
						
						
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

			def armingDrone():
				
				vehicle.parameters['ARMING_CHECK']=0

				time.sleep(0.1)
				print "Arming check status:  %s" % vehicle.parameters['ARMING_CHECK']

				print "Arming motors"
				vehicle.mode    = VehicleMode("GUIDED")
				vehicle.armed   = True
				vehicle.flush()

				while not vehicle.armed:
					vehicle.armed   = True
					print " Waiting for arming..."
					time.sleep(1)

				if vehicle.armed == True:
					print "Copter is armed"
					
			def printAttributes():
				#print "Global Location (relative altitude): %s" % vehicle.location.global_relative_frame
				print "Altitude relative to home_location: %s" % vehicle.location.global_relative_frame.alt
				#print " Local Location: %s" % vehicle.location.local_frame
				print " Battery: %s" % vehicle.battery
				print " Attitude: %s" % vehicle.attitude
				
			def printVelocity():
				
				#print "Velocity in Vx Vy Vz %s" %vehicle.velocity 
				vehicleVelocity = str(vehicle.velocity)
				vehicleVelocity = re.findall(r"-?\d.\d+", vehicleVelocity)
				Vx = float (vehicleVelocity[0])
				Vy = float (vehicleVelocity[1])
				Vz = float (vehicleVelocity[2])
				Vtotal = math.sqrt(Vx*Vx+Vy*Vy+Vz*Vz)
				print "Total velocity: %.3f" % Vtotal +" m/s"
				
				print "Flow X: ", self.flowX
				print "Flow Y: ", self.flowY
				print "Flow Vx: ", self.Vx
				print "Flow Vy: ", self.Vy 
				
				pubVelocity.publish(Vtotal)
				
				#return Vtotal

			def modeChange(mode): 
				print "Changing the vehicle mode to %s" % mode
				vehicle.mode    = VehicleMode(mode)
				vehicle.flush()

				while (vehicle.mode != mode):
					print "Changing mode ......"
					vehicle.mode    = VehicleMode(mode)
					time.sleep(1)

				print "Mode has been changed to %s " %mode 
				
			def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    
				msg = vehicle.message_factory.set_position_target_global_int_encode(
					0,       # time_boot_ms (not used)
					0, 0,    # target system, target component
					mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
					0b0000111111000111, # type_mask (only speeds enabled)
					0, # lat_int - X Position in WGS84 frame in 1e7 * meters
					0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
					0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
					# altitude above terrain if GLOBAL_TERRAIN_ALT_INT
					velocity_x, # X velocity in NED frame in m/s
					velocity_y, # Y velocity in NED frame in m/s
					velocity_z, # Z velocity in NED frame in m/s
					0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
					0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

				# send command to vehicle on 1 Hz cycle
				for x in range(0,duration):
					vehicle.send_mavlink(msg)
					time.sleep(1)    


			def condition_yaw(heading, relative=False):
			   
				if relative:
					is_relative = 1 #yaw relative to direction of travel
				else:
					is_relative = 0 #yaw is an absolute angle
				# create the CONDITION_YAW command using command_long_encode()
				msg = vehicle.message_factory.command_long_encode(
					0, 0,    # target system, target component
					mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
					0, #confirmation
					heading,    # param 1, yaw in degrees
					0,          # param 2, yaw speed deg/s
					1,          # param 3, direction -1 ccw, 1 cw
					is_relative, # param 4, relative offset 1, absolute angle 0
					0, 0, 0)    # param 5 ~ 7 not used
				# send command to vehicle
				vehicle.send_mavlink(msg)


			def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
				
				msg = vehicle.message_factory.set_position_target_local_ned_encode(
					0,       # time_boot_ms (not used)
					0, 0,    # target system, target component
					#mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame relative to earth's heading
					mavutil.mavlink.MAV_FRAME_BODY_NED, #frame relative to the nose's heading 
					0b0000111111000111, # type_mask (only speeds enabled)
					0, 0, 0, # x, y, z positions (not used)
					velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
					0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
					0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)


				# send command to vehicle on 1 Hz cycle
				for x in range(0,duration):
					vehicle.send_mavlink(msg)
					time.sleep(1)

			def takeOff(aTargetAltitude):
				print "Taking off!"
				vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude
				# Check that vehicle has reached takeoff altitude
				while True:
					
					printVelocity()
					print " Altitude: ", vehicle.location.global_relative_frame.alt 
					#Break and return from function just below target altitude.  
					
					"""
					if self.groundDistance >=aTargetAltitude*0.95:
						print "Reached target altitude"
						break
						
					"""  
					if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
					  print "Reached target altitude"
					  break
					time.sleep(1)
					
			def get_location_metres(original_location, dNorth, dEast):
				
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
				
				
			def get_bearing(aLocation1, aLocation2):
				"""
				Returns the bearing between the two LocationGlobal objects passed as parameters.

				This method is an approximation, and may not be accurate over large distances and close to the
				earth's poles. It comes from the ArduPilot test code:
				https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
				"""
				off_x = aLocation2.lon - aLocation1.lon
				off_y = aLocation2.lat - aLocation1.lat
				bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
				if bearing < 0:
					bearing += 360.00
				return bearing;


			def get_distance_metres(aLocation1, aLocation2):
			   
				dlat = aLocation2.lat - aLocation1.lat
				dlong = aLocation2.lon - aLocation1.lon
				return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


			def goto(gotoFunction=vehicle.simple_goto):
				
				
				while vehicle.mode.name=="GUIDED":
					
					"""		
					
					"""
					
					if self.frequency == 500:
						self.mavlinkCheck = ''
						self.frequency = 0 
					
					print "Current latitude of teddy bear: ", self.latitude 
					print "Current longitude of teddy bear: ", self.longitude 
				
									
					targetLocation = LocationGlobalRelative(self.latitude, self.longitude,0)
									
					currentLocation=vehicle.location.global_relative_frame
					#targetLocation=get_location_metres(currentLocation, dNorth, dEast)
					
					targetDistance=get_distance_metres(currentLocation, targetLocation)
					
					targetBearing=get_bearing(currentLocation, targetLocation) 
					
					print "Bearing: ", targetBearing 
					
					print "Type of bearing: ", type(targetBearing)
					
					#or only send the location when there is a difference in distance between two points
					if self.mavlinkCheck != 'goto':
						gotoFunction(targetLocation)
						self.mavlinkCheck = 'goto'
						

					print "Current altitude: %s" % vehicle.location.global_relative_frame.alt
					vehicle.groundspeed = 2
					vehicle.airspeed = 2
					remainingDistance=get_distance_metres(vehicle.location.global_frame, targetLocation)
					print "Distance to target: ", remainingDistance
					printVelocity()
					
					
					print "Value of self.obstacleStatus %s " % self.obstacle
					
					self.frequency += 1
					print "Frequency: ", self.frequency 
					
					if self.obstacle != "NONE" : 
						
						#////////////////////// obstacle avoidance algorithm ///////////////////////////
						#//////////////////////////////////////////////////////////////////////////////
						print "Value of self.obstacleStatus: %s"%self.obstacle
						brakeStatus = 0
						while True:
							if brakeStatus == 0:
								modeChange("BRAKE")
								time.sleep(2)
								brakeStatus +=1
								modeChange("GUIDED")
							print "Obstacle detected, flying upwards"
							printVelocity()
							send_ned_velocity(0,0,-0.2,1)
							print "It has rised till: %s" % vehicle.location.global_relative_frame.alt
							time.sleep(0.1)
							if self.obstacle == "NONE":
								print "Obstacle cleared"
								print "New altitude: %s" % vehicle.location.global_relative_frame.alt
								send_ned_velocity(0,0,0,0)
								vehicle.groundspeed = 2
								gotoFunction(targetLocation)
								self.frequency = 0 
								break; 
								
					#///////////////// similarly if I want to fly sideways ///////////////////////////////
					
					
					
				
					
					#//////////////////////////////////////////////////////////////////////////////////////
								
								
					#////////////////////////// end of algorithm /////////////////////////////////////////
					#/////////////////////////////////////////////////////////////////////////////////////
					
					#//////////////////////////// checking for remaining distance ////////////////////////
								
					if remainingDistance<=targetDistance*0.02: #Just below target, in case of undershoot.
						print "Reached target"
						break;
						
					self.rate.sleep()
						
					 
					#time.sleep(0.1)
					#remove this comment if I want to adjust the looping speed 
					
					#////////////////////////////////// end of goto() //////////////////////////////////////
					#/////////////////////////////////////////////////////////////////////////////////////////
					
			#///////////////////////////////////////////////////////////////////
			#///////////////////// Checking for GPS Fix ////////////////////////
			
			while vehicle.gps_0.fix_type < 2:
				print "Waiting for GPS ......." , vehicle.gps_0.fix_type
				time.sleep(3)
				
			#///////////////////////////////////////////////////////////////////
			#///////////////////////// GPS Fixed ///////////////////////////////
			
			#in the future I can check for number of satellites to improve on the signal 


			#///////////////////////// ARMING //////////////////////////////////
			#//////////////////////////////////////////////////////////////////
			
			armingDrone()

			#///////////////////////////////////////////////////////////////////
			#///////////////////////// ARMED ///////////////////////////////////

				
			printAttributes()

			#/////////////// Prepare for take off correction for goto() ////////
			#//////////////////////////////////////////////////////////////////
			
			# ------------------ obsolete ---------------------------------------
			print " Attitude: %s" % vehicle.attitude
			vehicleAttitude = str (vehicle.attitude)
			vehicleAttitude = re.findall(r"-?\d.\d+", vehicleAttitude)
			startupPitch = vehicleAttitude[0]
			startupYawStr = vehicleAttitude[1]
			startupRoll = vehicleAttitude[2]
			print "Startup pitch: %s" % startupPitch
			print "Startup yaw: %s" % startupYawStr
			print "Startup roll %s" % startupRoll 
			startupYaw = float(startupYawStr)
			#///////////////////////////////////////////////////////////////////



			#///////////////////////////////////////////////////////////////////
			#////////////////////////// Taking Off /////////////////////////////

			targetAltitude = 2
			print ("Taking off to %s" %targetAltitude +" meter")
			a = datetime.datetime.now()
			takeOff(targetAltitude)
			print "Altitude after take off:%s" % vehicle.location.global_relative_frame.alt
			b = datetime.datetime.now()
			print "Time it took to take off: %s" % (b-a)


			#///////////////////////////////////////////////////////////////////
			#////////////////// Take Off Complete //////////////////////////////

		
			#////////////////// if yawing is required ///////////////////////////
			
			
			
			#////////////////////// executing goto() ////////////////////////////
			#///////////////////////////////////////////////////////////////////
			
			
			# ---------------------- obsolete ----------------------------------
			"""
			desiredDistance = 50 
				
			print "Value of x component: ", math.cos(startupYaw)
			print "Value of y component: " , math.sin(startupYaw)

			dNorth = desiredDistance* math.cos(startupYaw)
			dEast = desiredDistance* math.sin(startupYaw)

			print "Distance towards the north: " ,dNorth
			print "Distance towards the east: " ,dEast

			vehicle.groundspeed = 2

			calculatedDistance = math.sqrt(math.pow(dNorth,2) + math.pow(dEast,2))
			print "Calculated distance in metres: " , calculatedDistance
			"""
			
			#///////////////////////////////////////////////////////////////////

			
			#////////////////// finding the teddy bear /////////////////////////
			
			goto()
			#/////////////////////////////////////////////////////////////////	  
			#////////////////// Target Distance Reached /////////////////////


			#//////////////////////////////////////////////////////////////////////
			#//////////////////////// END/ LAND ///////////////////////////////////

			
			modeChange("LAND")
			
			

			# Close vehicle object
			vehicle.close()
			
			break

	
if __name__ == '__main__':
	listener()
