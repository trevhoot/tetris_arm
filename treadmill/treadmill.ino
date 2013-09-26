

int inputPin = A0; // define analog input pin
int outputPin = D0; // define digital output pin for custom duty cycle
int potVal = 0; // define value to store potentiometer reading
int highDuty = 0; // define time to remain high in custom duty cycle


void main() {
  potVal = analogRead(inputPin);
  highDuty = map(potVal, 0, 255, 2500, 15500);
  digitalWrite(outputPin, 1);
  delayMicroseconds(highDuty);
  digitalWrite(outputPin, 0);
  delayMicroseconds(16000 - highDuty);
}
