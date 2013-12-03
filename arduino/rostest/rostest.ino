#include <Servo.h> 
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>

Servo gripperServo;  
Servo tread;


String data = String("Hello");

std_msgs::String tick_msg;
std_msgs::String gripper_msg;
ros::Publisher chatter("encoderTick", &tick_msg);
ros::Publisher gripperPub("actuated", &gripper_msg);

int encoderPin = 2;
boolean switchState = false;
boolean reading;

void gripperCb( const std_msgs::String& gripperSize){
  data = gripperSize.data;
  if (data == "small") {
     gripperServo.write(20);                 
     delay(15);
     digitalWrite(13,HIGH);
   }
   
   if (data == "big") {
     gripperServo.write(70);                  
     delay(15);
     digitalWrite(13,LOW);
   }
    
   if (data == "open") {
     gripperServo.write(120);                 
     delay(15);
   }
   
}

void treadmillCb( const std_msgs::String& motorSpeed){
   data = motorSpeed.data;
   if (data == "go"){
    tread.writeMicroseconds(1800);
    delay(50); 
   }
   
   if (data == "stop"){
     tread.writeMicroseconds(1400);
     delay(50);
   }
}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::String> gripperSub("gripperSize" , gripperCb);
ros::Subscriber<std_msgs::String> treadmillSub("treadmillMotor" , treadmillCb);


void setup() 
{ 
  tread.attach(9);
  gripperServo.attach(8); 
  nh.initNode();
  nh.subscribe(gripperSub);
  nh.subscribe(treadmillSub);
  nh.advertise(chatter);

  
} 
 
void loop() 
{ 
  nh.spinOnce();
  delay(1); 

  reading = digitalRead(encoderPin);

  if (reading && switchState){
    switchState = false;
    tick_msg.data = "ping";
    chatter.publish(&tick_msg);
  }
  else if(!reading){
    switchState = true;
  }


} 
