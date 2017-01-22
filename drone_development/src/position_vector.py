#!/usr/bin/env python
from dronekit import LocationGlobal
import math 

position_vector_home_location = LocationGlobal(0,0,0)
# scaling used to account for shrinking distances between longitudinal lines as we move away from the equator 
# not sure if I understood the formula but it has tested to be working fine 
position_vector_lon_scale = 1.0
# converts lat and lon to metres 
position_vector_latlon_to_metres = 111319.5

class PositionVector(object):
	def __init__(self):
		pass
		
	def __str__(self):
		pass
	
	@classmethod
	def set_home_location(cls,home_location):
		# trying out @classmethod 
		# tested to be working 
		global position_vector_home_location
		position_vector_home_location = home_location
		print 'Home location: ', position_vector_home_location
		PositionVector.update_lon_scale(position_vector_home_location.lat)
		
	@classmethod
	def update_lon_scale (cls, lat):
		global position_vector_lon_scale
		if lat <> 0:
			position_vector_lon_scale = math.cos(math.radians(lat))
		
		
		
		
