
#include <Servo.h> 

Servo motor;		// create servo object to control the motor speed
Servo gripper;  	// create servo object to control a the gripper 

int inputPin = A0;	// define analog input pin
int motorPin = 9; 	// define digital output pin to motor
int gripperPin = 10;	// define digital output pin to gripper
int gripAngle = 0;      // define incoming angle command from the computer

void setup() {
  motor.attach(9);  	// attaches the servo on pin 9 to the servo object 
  gripper.attach(10);  	// attaches the gripper on pin 9 to the servo object 

  Serial.begin(9600);
}

void loop() {
  gripAngle = Serial.read();
  gripper.write(gripAngle);
  Serial.write('gripAngle'); 
  delayMicroseconds(100);
  motor.writeMicroseconds(1200);
  
}



