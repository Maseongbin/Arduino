#include <MsTimer2.h>

//#define A0pin A0

#define NO_BUFFER_SEC 100
#define MAX_SEC        10

int *data;
int i = 0;
bool buffer_full_front = false;
bool buffer_full_rear  = false;
bool bank_no           = false;
void setup() 

{
  data =(int *)malloc(sizeof(int) * NO_BUFFER_SEC * MAX_SEC * 2);
  i = 0;
  MsTimer2::set(1000, timerISR);
  MsTimer2::start();
  Serial.begin(115200);
}

void timerISR()
{
  data[i] = analogRead(A0);
  i++;
  
  if(i < NO_BUFFER_SEC * MAX_SEC)
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
  unsigned long data_size;

  data_size = sizeof(data) / sizeof(int);
  Serial.println(data_size);
}
