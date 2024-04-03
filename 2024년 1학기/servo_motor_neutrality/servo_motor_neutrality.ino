#include <Servo.h>

Servo steeringServo;  
int steeringPin = 8;  

void setup() 
{
  steeringServo.attach(steeringPin);  
}

void loop() 
{
  steeringServo.write(90);
  delay(1000); 
}
