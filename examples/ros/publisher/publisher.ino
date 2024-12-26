#include <VBCoreG4_arduino_system.h>

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
  pinMode(LED2, OUTPUT);
}

void loop()
{ 
  digitalWrite(LED2, HIGH);   // turn the LED on (HIGH is the voltage level)
                    
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(1000);                    // wait for a second
  digitalWrite(LED2, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
}
