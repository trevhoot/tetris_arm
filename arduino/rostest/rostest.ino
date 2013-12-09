#include <Servo.h> 
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>

Servo gripperServo;  
Servo tread;


String data = String("Hello");

std_msgs::String tick_msg;
std_msgs::String gripper_msg;
std_msgs::String print_msg;
ros::Publisher chatter("encoderTick", &tick_msg);
ros::Publisher gripperPub("actuated", &gripper_msg);
ros::Publisher printOut("print", &print_msg);

int encoderPin = 2;
boolean switchState = false;
boolean reading;

void gripperCb( const std_msgs::String& gripperSize){
  data = gripperSize.data;
  printOut.publish(&print_msg);
  if (data == "open") {
     print_msg.data = "adruino.gripperCb: recieved /gripperSize open";
     printOut.publish(&print_msg);
     gripperServo.write(165);                 
     delay(15);
     digitalWrite(13,HIGH);
   }
   
   if (data == "small") {
     print_msg.data = "adruino.gripperCb: recieved /gripperSize small";
     printOut.publish(&print_msg);
     gripperServo.write(90);                  
     delay(15);
     digitalWrite(13,LOW);
   }
    
   if (data == "big") {
     print_msg.data = "adruino.gripperCb: recieved /gripperSize big";
     printOut.publish(&print_msg);
     printOut.publish(&print_msg);
     gripperServo.write(125);                 
     delay(15);
   }
   tick_msg.data = "ping";
   chatter.publish(&tick_msg);
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
  gripperServo.write(165);  
  nh.initNode();
  nh.subscribe(gripperSub);
  nh.subscribe(treadmillSub);
  nh.advertise(chatter);
  nh.advertise(printOut);

  
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
