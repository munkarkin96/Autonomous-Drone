from pymavlink import mavutil #needed for all MAV_MESSAGES 
import time #for time.sleep()

class mavlinkMessages (object):
	
	def __init__(self, dronekit_vehicle):
		self.vehicle = dronekit_vehicle 
		
		
	def condition_yaw (self, heading, relative=False):
		if relative:
			is_relative = 1 #yaw relative to direction of travel
		else:
			is_relative = 0 #yaw is an absolute angle
		# create the CONDITION_YAW command using command_long_encode()
		msg = self.vehicle.message_factory.command_long_encode(
			0, 0,    # target system, target component
			mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
			0, #confirmation
			heading,    # param 1, yaw in degrees
			0,          # param 2, yaw speed deg/s
			1,          # param 3, direction -1 ccw, 1 cw
			is_relative, # param 4, relative offset 1, absolute angle 0
			0, 0, 0)    # param 5 ~ 7 not used
		# send command to vehicle
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()


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

				
	def send_ned_velocity (self, Vx, Vy, Vz, duration):
		print "sending NED velocity"
		msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
				0,       # time_boot_ms (not used)
				0, 0,    # target system, target component
				#mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame relative to earth's heading
				mavutil.mavlink.MAV_FRAME_BODY_NED, #frame relative to the nose's heading 
				0b0000111111000111, # type_mask (only speeds enabled)
				0, 0, 0, # x, y, z positions (not used)
				Vx, Vy, Vz, # x, y, z velocity in m/s
				0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
				0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

		# send command to vehicle on 1 Hz cycle
		for x in range(0,duration):
			self.vehicle.send_mavlink(msg)
			self.vehicle.flush()
			time.sleep(1)
			
			
	def send_land_message(self, x, y, horizontal_resolution, horizontal_fov, vertical_resolution, vertical_fov):
		print 'self.vehicle.rangefinder.distance: ',self.vehicle.rangefinder.distance 
		msg = self.vehicle.message_factory.landing_target_encode(
			0,       # time_boot_ms (not used)
			0,       # target num
			0,       # frame
			(x-horizontal_resolution/2)*horizontal_fov/horizontal_resolution,
			(y-vertical_resolution/2)*vertical_fov/vertical_resolution,
			0,       # altitude.  Not supported.
			0,0)     # size of target in radians
		print "sending land_message" 
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()		
			
		
	def distance_sensor(self, ground_distance):
		# publish MAV_DISTANCE_SENSOR to pixhawk 
		self.vehicle.parameters['RNGFND_TYPE'] = 10 
		min_distance = 20
		max_distance = 1600
		
		msg = self.vehicle.message_factory.distance_sensor_encode(
			0, #time-boost-ms (not used)
			min_distance, #centimeters
			max_distance, #cemtimeters 
			ground_distance, #centimeters
			mavutil.mavlink.MAV_DISTANCE_SENSOR_INFRARED, 
			0, #id (not used)
			25, #orientation (i.e PITCH_270) 
			255) #not used  
		self.vehicle.send_mavlink(msg)	
							
		#print "vehicle.rangefinder  ----------- publishing distance_sensor: ", self.vehicle.rangefinder.distance #returns float
		

	def stabilize_gimbal(self):
		# FIXME not tested 
		msg = self.vehicle.message_factory.mount_configure_encode(
			0, #target system
			1, #target component
			mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING, 
			#mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_GPS_POINT,
			1, #stabilize roll 
			1, #stabilize pitch 
			1, #stabilize roll (or 0) 
			)
		self.vehicle.send_mavlink(msg)
		self.vehicle.flush()
		
		
	def point_gimbal(self):
		
		'''
		#remember to convert to degrees if the reading is in radians 	
		#not sure if I need to run stabilize_gimbal() before running point_gimbal() yet 
		
		----- for HOBBY SIGLO 3-axis Brushless Camera Gimbal ------
		
		roll range: -90 to 90 
		pitch range: -45 to 45 
		yaw range: -90 to 90 
		
		------------------------------------------------------------	
		'''
		
		pitch = 0 
		roll = 0 
		yaw = 0 
		
		# FIXME not tested 
		msg = self.vehicle.message_factory.mount_control_encode(
			0, 1, #target system, component
			pitch*100, #must be in degrees *100 
			roll*100,
			yaw*100,
			0) #save position
			



