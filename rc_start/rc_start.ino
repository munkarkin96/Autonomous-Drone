#include <ros.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

unsigned long rc_ch7;
int ch7_in = 7; 

ros::NodeHandle nh;

std_msgs::Float32 str_msg;
ros::Publisher chatter("arduinoMessage", &str_msg);


void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  pinMode (ch7_in, INPUT);
  Serial.begin(9600);
}

void loop()
{
  rc_ch7 = pulseIn(ch7_in, HIGH);
  rc_ch7 = 1000;
  str_msg.data = rc_ch7;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);
}

