#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32
import time


class listener (object):

	def __init__(self):	
		rospy.init_node('arduinoListener', anonymous=True)
		rospy.Subscriber('Location', String, self.callback)

		self.Location = ''
		locationLatitude = 0
		locationLongitude = 0 
		
		locationLatitudeStr = '0' 
		locationLongitudeStr = '0' 
		
		self.start_time = time.time()

		while True:		
			
							
			if (time.time() - self.start_time > 5):
					
				pubLatitude = rospy.Publisher("latitude", Float32, queue_size = 10)
				pubLongitude = rospy.Publisher("longitude", Float32, queue_size = 10)
				
							
				if self.Location == "Latitude: ":
					while self.Location == "Latitude: ":
						time.sleep(0.1)
						
					locationLatitudeStr = self.Location
					print "Latitude: %s" %locationLatitude 
					#pubLatitude.publish(locationLatitude)
					
					
				if self.Location =="Longitude: ":
					while self.Location == "Longitude: ":
						time.sleep(0.1)				
						
					locationLongitudeStr = self.Location
					print "Longitude: %s" %locationLongitude 
					#pubLongitude.publish(locationLongitude)
					
				locationLatitude = float (locationLatitudeStr)
				locationLongitude = float (locationLongitudeStr)
			
				pubLatitude.publish(locationLatitude)
				pubLongitude.publish(locationLongitude)

				#rate = rospy.Rate(10)
				#rate.sleep()
				
				
			
	def callback (self,arduinoData):
		self.Location = arduinoData.data
		
	
if __name__ == '__main__':
	listener()






			
		
	

	
