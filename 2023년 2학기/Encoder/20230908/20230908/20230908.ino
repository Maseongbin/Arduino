const byte outPin = 13; // Output pin: digital pin 13(D13)
const byte interruptPin1 = 2; // Interrupt pin: D2
const byte interruptPin2 = 3; // Interrupt pin: D2
const int encoder1_A = 18;
const int encoder1_B = 19;
const int encoder2_A = 20;
const int encoder2_B = 21;
const byte resetPin = 5;
volatile byte state = 0;
long cnt1 = 0; // 추가
long cnt2 = 0; // 추가

void setup()
{
  pinMode(outPin, OUTPUT); // Output mode
  pinMode(interruptPin1, INPUT_PULLUP); // Input mode, pull-up
  pinMode(interruptPin2, INPUT_PULLUP); // Input mode, pull-up
  pinMode(encoder1_A, INPUT_PULLUP);
  pinMode(encoder1_B, INPUT_PULLUP);
  pinMode(encoder2_A, INPUT_PULLUP);
  pinMode(encoder2_B, INPUT_PULLUP);
  pinMode(resetPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoder1_A), intfunc1, RISING); // Enable interrupt
  attachInterrupt(digitalPinToInterrupt(encoder2_A), intfunc2, RISING); // Enable interrupt
  Serial.begin(115200);
}

void intfunc1() // Interrupt function
{
  if (digitalRead(encoder1_B) == HIGH) // If D4 output is low
  {
    cnt1++;
  }
  else
  {
    cnt1--;
  }
}

void intfunc2() // Interrupt function
{
  if (digitalRead(encoder2_B) == LOW) // If D4 output is low
  {
    cnt2++;
  }
  else
  {
    cnt2--;
  }
}

void loop()
{
  Serial.print("cnt1: ");
  Serial.print(cnt1);

  Serial.print("       cnt2: ");
  Serial.println(cnt2);
  Serial.println();

  delay(300);
}
