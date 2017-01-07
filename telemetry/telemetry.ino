#include <SoftwareSerial.h>
#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("Location", &str_msg);

SoftwareSerial mySerial(10, 11); //TX, RX
// gnd SET_A and SET_B for Normal Mode (Send and Receive)

void setup() {
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  
  nh.advertise(chatter);
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {

  if(mySerial.available() > 1){//Read from UM402 and send to serial monitor
    String incomingLocation = mySerial.readString();
    Serial.println(incomingLocation);  
    
    int stringLength = incomingLocation.length() + 1;
    
    char charArray[stringLength];
    incomingLocation.toCharArray(charArray, stringLength);
  
    str_msg.data = charArray;
    chatter.publish(&str_msg);
    
     
  }
  delay(20);
  nh.spinOnce();
}
