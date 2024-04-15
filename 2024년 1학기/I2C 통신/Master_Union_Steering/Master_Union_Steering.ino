
#include <Servo.h>
#include <Wire.h>

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
  Wire.begin();
 
  Serial.begin(115200);
}

void loop()
{  
  Serial.print("mbyte[0]: ");
  while (!Serial.available()) {}
  u.mbyte[0] = Serial.parseInt();
  Serial.println(u.mbyte[0]);
  Serial.read();
  
  Serial.print("mbyte[1]: ");
  while (!Serial.available()) {}
  u.mbyte[1] = Serial.parseInt();
  Serial.println(u.mbyte[1]);
  Serial.read();
  
  Wire.beginTransmission(1);
  Wire.write(u.mbyte, 2);      
  Wire.endTransmission();

  delay(1000);
}
