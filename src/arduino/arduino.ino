#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

int led = 12;  

std_msgs::String str_msg;

ros::Publisher chatter("Navigator/arduinoResponse", &str_msg);

void messageCb( const std_msgs::String& arduinoCommand){
  chatter.publish( &arduinoCommand );
  nh.spinOnce();
}

ros::Subscriber<std_msgs::String> sub("Navigator/arduinoCommands", &messageCb );

void setup()
{
  pinMode(led, OUTPUT); 
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}


