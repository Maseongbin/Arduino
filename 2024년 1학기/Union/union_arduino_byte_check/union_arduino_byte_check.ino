void setup() 
{
  Serial.begin(115200);

  Serial.print("Size of int: ");
  Serial.println(sizeof(int));

  Serial.print("Size of float: ");
  Serial.println(sizeof(float));

  Serial.print("Size of double: ");
  Serial.println(sizeof(double));

  Serial.print("Size of char: ");
  Serial.println(sizeof(char));
}

void loop() 
{

}
