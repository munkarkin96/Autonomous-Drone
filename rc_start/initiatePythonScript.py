import os 
import RPi.GPIO as GPIO
import time
import serial
from binascii import hexlify

ser = serial.Serial('/dev/ttyACM0',9600)
#s = [0,1]

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

#adjust for where your switch is connected
buttonPin = 18
GPIO.setup(buttonPin,GPIO.OUT)

def redLED_ON ():
	print "red LED indicator is turned ON"
	GPIO.output(18,GPIO.HIGH)
	time.sleep(3)

def redLED_OFF ():
	print "red LED indicator is turned OFF"
	GPIO.output(18,GPIO.LOW)
	time.sleep(1)

def serialAvailable():
	
	readSerial=ser.readline()
	#s[0] = str(int (ser.readline(),36))
	#print s[0]
	#print readSerial still in hexa format
	readSerial_int = int(readSerial, 16)
	
	if (readSerial_int > 800 and readSerial_int < 2200):

		#readSerial = str(readSerial)
		print "Value: %s" % readSerial_int
			
		if readSerial_int <1000:
			#do nothing 
			return 0
		elif readSerial_int > 1800:
			#initiate python script 
			return 1

print "Python sript initiator has been invoked."

while True:
	pythonStatus = serialAvailable()
	if (pythonStatus == 1):
		redLED_OFF()
		print ("Initiating command line python takeOffLand.py --connect /dev/ttyS0")
		os.system ('python takeOffLand.py --connect /dev/ttyS0')
		#os.system ('python templateTest.py --connect "udp:192.168.1.5:14550"')
		print ("Python startup failed/ script has completed")
		redLED_ON()	
	
	else: 
		print ("Nothing has been initiated")
		
	#elif #used for different scripts in the future 









			
		
	

	
