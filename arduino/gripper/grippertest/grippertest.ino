
#include <Servo.h> 
 
Servo myservo;   
 
char incomingByte = 'b';

void setup() 
{ 
  Serial.begin(9600);
  myservo.attach(8);  
} 
 
void loop() 
{ 
  if (Serial.available() > 0) {
    incomingByte = Serial.read();
    
 
    if (incomingByte == '2') {
      myservo.write(0);                 
      delay(15);
      digitalWrite(13,HIGH);
    }
  
    if (incomingByte == '1') {
      myservo.write(90);                  
      delay(15);
      digitalWrite(13,LOW);
    }
  
    if (incomingByte == '0') {
      myservo.write(180);                 
      delay(15);
    }
    
    Serial.write(incomingByte);
    
    
    
  
  }
     
} 
