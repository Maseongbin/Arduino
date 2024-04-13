union UNION 
{
  int angle;
  unsigned char mbyte[2];
} u;

void setup()
{
  Serial.begin(115200);
}

void loop()
{
  u.angle = -30;
  
  Serial.print("mbyte[0]: ");
  Serial.println(u.mbyte[0]);
  
  Serial.print("mbyte[1]: ");
  Serial.println(u.mbyte[1]);
  delay(1000);
}
