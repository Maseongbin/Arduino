#include <Wire.h>

void setup() 
{
  Wire.begin(1);                
  Wire.onRequest(requestEvent); 
  Wire.onReceive(receiveEvent); 

  Serial.begin(9600);           
}

void loop() 
{
  delay(500);
}

void receiveEvent(int howMany) 
{ 
  while (Wire.available()>1) 
  { 
    char ch = Wire.read(); 
    Serial.print(ch);         
  }
  int x = Wire.read();    
  Serial.println(x);      
}
void requestEvent() 
{ 
  Wire.write("ok!\n");   
}
