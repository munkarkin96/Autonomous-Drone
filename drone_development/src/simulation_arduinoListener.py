#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import time


class listener (object):

	def __init__(self):	
		rospy.init_node('arduinoListener', anonymous=True)
		#rospy.Subscriber('Location', String, self.callback)

		#self.Location = ''
		locationLatitude = 0
		locationLongitude = 0 
		
		locationLatitudeStr = '0' 
		locationLongitudeStr = '0' 
		
		self.start_time = time.time()
		
		self.threshold_seconds = 25
		self.threshold_limit = 50
		self.current_time = time.time()
		
		pubLatitude = rospy.Publisher("latitude", Float32, queue_size = 10)
		pubLongitude = rospy.Publisher("longitude", Float32, queue_size = 10)
				

		while True:		
			'''
			if (time.time() - self.current_time < self.threshold_seconds):
				locationLatitudeStr = '1.309006'
				locationLongitudeStr = '103.775324' 
				locationLatitude = float (locationLatitudeStr)
				locationLongitude = float (locationLongitudeStr)
				
			if (time.time() - self.current_time > self.threshold_seconds):
				locationLatitudeStr = '1.309915'
				locationLongitudeStr = '103.776068' 
				locationLatitude = float (locationLatitudeStr)
				locationLongitude = float (locationLongitudeStr)
				
			if (time.time() - self.current_time > self.threshold_limit):
				self.current_time = time.time()
			'''
			locationLatitudeStr = '1.309006'
			locationLongitudeStr = '103.775324'
			locationLatitude = float (locationLatitudeStr)
			locationLongitude = float (locationLongitudeStr)
			
			pubLatitude.publish(locationLatitude)
			pubLongitude.publish(locationLongitude)
			time.sleep(5)
'''				
			
	def callback (self,arduinoData):
		self.Location = arduinoData.data
'''		
	
if __name__ == '__main__':
	listener()






			
		
	

	
