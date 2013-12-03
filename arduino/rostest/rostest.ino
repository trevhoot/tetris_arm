#include <Servo.h> 
#include <ros.h>
#include <std_msgs/String.h>
#include <ros/time.h>

Servo myservo;  
Servo tread;


String data = String("Hello");

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);


void messageCb( const std_msgs::String& gripperSize){
  data = gripperSize.data;
  if (data == "open") {
     myservo.write(0);                 
     delay(15);
     digitalWrite(13,HIGH);
   }
   
   if (data == "middle") {
     myservo.write(95);                  
     delay(15);
     digitalWrite(13,LOW);
   }
    
   if (data == "close") {
     myservo.write(180);                 
     delay(15);
   }
   
   if (data == "go"){
    tread.writeMicroseconds(1800);
    delay(50); 
   }
   
   if (data == "stop"){
     tread.writeMicroseconds(1400);
     delay(50);
   }
  
   memcpy(&str_msg.data, millis(), sizeof(millis()));
   //str_msg.data = (char*)millis();
   chatter.publish(&str_msg);
}

ros::NodeHandle  nh;
ros::Subscriber<std_msgs::String> sub("gripperSize" , messageCb);

void setup() 
{ 
  tread.attach(9);
  myservo.attach(8); 
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(chatter);

  
} 
 
void loop() 
{ 
  nh.spinOnce();
  delay(1); 
} 
