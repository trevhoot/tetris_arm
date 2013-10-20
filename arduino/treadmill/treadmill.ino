

int inputPin = A0; // define analog input pin
int outputPin = 9; // define digital output pin for custom duty cycle
int potVal = 0; // define value to store potentiometer reading
int highDuty = 0; // define time to remain high in custom duty cycle
int outPin = 8;                 // digital pin 8

void setup() {
  // declare the outputPin as an OUTPUT
  pinMode(outputPin, OUTPUT);
  
  pinMode(outPin, OUTPUT);      // sets the digital pin as output
  Serial.begin(9600);
}

void loop() {
  potVal = analogRead(inputPin);
  highDuty = map(potVal, 0, 980, 2500, 16200);
  
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(highDuty);
  digitalWrite(outputPin, LOW);
  delayMicroseconds(16000 - highDuty);
/*
  digitalWrite(outPin, HIGH);   // sets the pin on
  delayMicroseconds(50);        // pauses for 50 microseconds      
  digitalWrite(outPin, LOW);    // sets the pin off
  delayMicroseconds(50);        // pauses for 50 microseconds      
*/
}

