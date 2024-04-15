#include <Wire.h>

void setup() 
{
  Wire.begin();
  Serial.begin(9600); 
}

byte x = 0;

void loop() 
{
  Wire.beginTransmission(1);                
  Wire.write("good ");       
  Wire.write(x);             
  Wire.endTransmission();    
     
  delay(500);
  
  Wire.requestFrom(1, 4);
  while (Wire.available())
  {
    char c = Wire.read(); 
    Serial.print(c);        
  }    
  x++;
  if(x==6)x=0;  
}
