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
  Serial.begin(115200);
  Steeringservo.attach(RC_SERVO_PIN);
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
  
  if(u.mbyte[0] == 30 && u.mbyte[1] == 0)
  {
    Steeringservo.write(NEUTRAL_ANGLE + u.angle);
  }
  
  if(u.mbyte[0] == 226 && u.mbyte[1] == 255)
  {
    Steeringservo.write(NEUTRAL_ANGLE + u.angle);
  }

  Serial.print("angle_offset: ");
  Serial.println(u.angle);
  Serial.print("angle: ");
  Serial.println(NEUTRAL_ANGLE + u.angle);
  Serial.println();
  delay(1000);
}
