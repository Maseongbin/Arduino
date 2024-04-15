#include <Wire.h>
#include <Servo.h>

#define NEUTRAL_ANGLE 90
#define RC_SERVO_PIN   8

int Steering_Angle = 0;
Servo Steeringservo;

union UNION 
{
  int angle;
  unsigned char mbyte[2];
} u;

void setup() 
{
  Wire.begin(1);               
  Wire.onReceive(receiveEvent);

  Serial.begin(115200);
  Steeringservo.attach(RC_SERVO_PIN);          
}

void loop() 
{
  delay(500);
}

void receiveEvent(int howMany) 
{ 
  u.mbyte[0] = Wire.read(); 
  u.mbyte[1] = Wire.read();
  
  Steeringservo.write(NEUTRAL_ANGLE + u.angle);
  
  Serial.print("angle_offset: ");
  Serial.println(u.angle);
  Serial.print("angle: ");
  Serial.println(NEUTRAL_ANGLE + u.angle);
  Serial.println();
  delay(1000);  
}
