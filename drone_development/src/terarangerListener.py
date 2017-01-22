#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import String
import time
from std_msgs.msg import Float32


class listener (object):

	def __init__(self):	
		rospy.init_node('teraListener', anonymous=True)
		rospy.Subscriber('terarangerone', Range, self.callback)
		rospy.Subscriber('terarangerone2', Range, self.callback2)
		rospy.Subscriber('terarangerone3', Range, self.callback3)	
		
		#rospy.Subscriber('currentVel', Float32, self.callbackVelocity)
		
		pub = rospy.Publisher('obstacleStatus', String, queue_size = 0)
		
		

		self.rate = rospy.Rate(400)
		self.tera1 = ''
		self.tera2 = ''
		self.tera3 = ''
		
		while True:
			
			print "Range of tera1: ", self.tera1
			print "Range of tera2: ", self.tera2 
			print "Range of tera3: ", self.tera3
						
			#safetyValue = 0.8 + self.currentVelocity 
			safetyValue = 3.5
			obstacleStatus = '' 
			
			
			if self.tera1 < safetyValue: 
				obstacleStatus = 'F'
			else:
				obstacleStatus = ''
				
				
			if self.tera2 < safetyValue:
				obstacleStatus += 'R'
			else: 
				obstacleStatus += ''
				
			if self.tera3 < safetyValue:
				obstacleStatus +='L' 
			else:
				obstacleStatus +=''
		
				
			if obstacleStatus == '':
				obstacleStatus = 'NONE'
				
			print "Obstacle status: %s" %obstacleStatus
			
			pub.publish(obstacleStatus)
			self.rate.sleep()
			
			
			
	def callback (self,teraData):
		self.tera1 = teraData.range
		
	def callback2(self, teraData):
		self.tera2 = teraData.range
		
	def callback3(self, teraData):
		self.tera3 = teraData.range
		
'''
	
	def callbackVelocity(self, velData):
		self.currentVelocity = velData.data
		
'''		
	
if __name__ == '__main__':
	listener()






			
		
	

	
