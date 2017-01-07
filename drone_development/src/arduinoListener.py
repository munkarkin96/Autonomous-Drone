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

		while True:		
			
							
				
			pubLatitude = rospy.Publisher("latitude", Float32, queue_size = 10)
			pubLongitude = rospy.Publisher("longitude", Float32, queue_size = 10)
						
			if self.Location == "Latitude: ":
				while self.Location == "Latitude: ":
					time.sleep(0.1)
					
				locationLatitude = self.Location
				print "Latitude: %s" %locationLatitude 
				#pubLatitude.publish(locationLatitude)
				
				
			if self.Location =="Longitude: ":
				while self.Location == "Longitude: ":
					time.sleep(0.1)				
					
				locationLongitude = self.Location
				print "Longitude: %s" %locationLongitude 
				#pubLongitude.publish(locationLongitude)
				
			locationLatitude = float (locationLatitude)
			locationLongitude = float (locationLongitude)
		
			pubLatitude.publish(locationLatitude)
			pubLongitude.publish(locationLongitude)

			#rate = rospy.Rate(10)
			#rate.sleep()
			
			
			
	def callback (self,arduinoData):
		self.Location = arduinoData.data
		
	
if __name__ == '__main__':
	listener()






			
		
	

	
