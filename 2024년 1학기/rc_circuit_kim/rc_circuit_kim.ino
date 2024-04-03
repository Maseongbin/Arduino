#define NO_BUFFER_SEC 100
#define MAX_SEC        10
#include <MsTimer2.h>

const int capacitorPin = A0;
unsigned long startChargingTime = 0;

int *data; 
int i = 0;
bool buffer_full_front = false;
bool buffer_full_rear  = false;
bool bank_no           = false;

void setup() 
{
  pinMode(capacitorPin, INPUT);
  Serial.begin(115200);
  data = (int *)malloc(sizeof(int)* NO_BUFFER_SEC * MAX_SEC * 2);
  MsTimer2::set(1000, timerISR);
  MsTimer2::start();
}

void timerISR()
{
  data[i] = analogRead(A0);
  i++;
  if(i< NO_BUFFER_SEC * MAX_SEC)
  {
    bank_no = false;
  }
  else
  {
    bank_no = true;
    if(i == NO_BUFFER_SEC * MAX_SEC * 2)
    {
      i = 0;
    }
  }
}

void loop()
{
  int Value = analogRead(capacitorPin);
  float voltage = (float)Value / 1023.0 * 5.0;

  Serial.print("Charging Voltage: ");
  Serial.print(voltage, 2);
  Serial.println(" V");
  
  pinMode(capacitorPin, OUTPUT);
  digitalWrite(capacitorPin, LOW);
  pinMode(capacitorPin, INPUT);  

  /*
  unsigned long data_size;
  data_size = sizeof(data) / sizeof(int);
  
  Serial.print("Data Size: ");
  Serial.println(data_size);
  Serial.println();
  */
}
