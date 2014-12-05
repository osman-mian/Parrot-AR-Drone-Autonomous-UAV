// Ultrasonic - Library for HR-SC04 Ultrasonic Ranging Module.
// Rev.4 (06/2012)
// J.Rodrigo ( www.jra.so )
// more info at www.ardublog.com

#include <Ultrasonic.h>
#include <ros.h>
#include <std_msgs/Int32.h>

Ultrasonic ultrasonic(12,13); // (Trig PIN,Echo PIN)

ros::NodeHandle nh;
//
std_msgs::Int32 pushed_msg;
ros::Publisher pub_button("Distance", &pushed_msg);

void setup() {
  Serial.begin(9600); 
  nh.initNode();
  nh.advertise(pub_button);
  pinMode(11, OUTPUT);
  digitalWrite(11,HIGH);
}

void loop()
{
  Serial.print(ultrasonic.Timing());
  Serial.print(" ms, " ); // milliseconds
  Serial.print(ultrasonic.Ranging(CM));
  Serial.println("cm");
  pushed_msg.data=ultrasonic.Ranging(CM);
  pub_button.publish(&pushed_msg);

  
  nh.spinOnce();
  delay(1000);
}

