#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
 
//int potpin = 0;  // analog pin used to connect the potentiometer
//int val;    // variable to read the value from the analog pin 
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  myservo.writeMicroseconds(1500);
} 
 
void loop() 
{ 
  myservo.writeMicroseconds(1000);
  delay(5000);
  myservo.writeMicroseconds(1200);
  delay(5000);
  myservo.writeMicroseconds(1500);
  delay(5000);
  myservo.writeMicroseconds(2000);
  delay(5000);
 // myservo.write(val);                  // sets the servo position according to the scaled value 
  //delay(15);                           // waits for the servo to get there 
} 
