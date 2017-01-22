
from os.path import expanduser #for self.config_file 
import ConfigParser 

class vehicle_config(object):
	
	
	def __init__(self):
		# FIXME need to change in raspberry pi as well (not done yet) 
		self.config_file = expanduser('~/catkin_ws/src/drone_development/src/vehicle_config_file.cnf')
		self.parser = ConfigParser.SafeConfigParser()
		self.read()
		
		
	def main(self):
		pass 
		# ---- reserved for future use ---- 
		
		
	def read(self):
		try:
			self.parser.read(self.config_file)
		except IOError as e:
			print 'Error {0} reading config file: {1}: '.format(e.errno, e.strerror)
		return
		
	
	def get_string(self, section, option, default):
		# get the connection_str 
		try:
			return self.parser.get(section, option)
		except ConfigParser.Error:
			return default 
			
			
	def get_integer(self, section, option, default):
		# get the baud rate 
		try:
			return self.parser.getint(section, option)
		except ConfigParser.Error:
			return default 
			
	def get_float(self, section, option, default):
		# to be used by the camera 
		try:
			return self.parser.getfloat(section, option)
		except ConfigParser.Error:
			return default
		

config = vehicle_config()


if __name__=='__main__':
	config.main()
