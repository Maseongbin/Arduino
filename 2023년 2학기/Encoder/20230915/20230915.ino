#include <MsTimer2.h>

#define wheel_track  0.13       //m 단위로 구할 것, 0.1 = 10cm
#define RAD2DEG(x)   (x*180.0/3.14159)
#define DEG2RAD(x)   (x*3.14159/180.0)

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
long cnt1_old = 0;
long cnt2_old = 0;

double pulse_to_distance_left  = 0.2/511;
double pulse_to_distance_right = 0.2/514;

const double odom_left  = 0;
const double odom_right = 0;
double yaw = 0.0;
double yaw_degree = 0.0;

double heading(double x, double y)
{
  double head = atan2(y, x); // stope Y, stope X
  return head;
}

void msTimer2_ISR()
{
  char msg[100] = {0x00};
  double odom_left_delta  = 0.0;
  double odom_right_delta = 0.0;
  double theta_delta = 0.0;
  double delta_encoder_right = 0.0;
  double delta_encoder_left = 0.0;
  double theta_delta_degree = 0.0;

  delta_encoder_right = cnt1 - cnt1_old;
  delta_encoder_left  = cnt2 - cnt2_old;
  
  //delta_encoder_right =  100;
  //delta_encoder_left  = -100;
  
  //sprintf(msg, "encorder delta : {%3d  %3d}", delta_encoder_left, delta_encoder_right);
  //Serial.println(msg);
  Serial.print("delta_encoder_right:  "); Serial.print(delta_encoder_right); Serial.print("  ");
  Serial.print("delta_encoder_left:  ");  Serial.print(delta_encoder_left); Serial.println("  ");
  Serial.println();
  
  odom_right_delta = (delta_encoder_right) * pulse_to_distance_right;
  odom_left_delta  = (delta_encoder_left)  * pulse_to_distance_left;
  Serial.print("odom_right_delta:  "); Serial.print(odom_right_delta); Serial.print("     ");
  Serial.print("odom_left_delta:  ");  Serial.print(odom_left_delta); Serial.println("    ");
  
  theta_delta = heading(wheel_track, (odom_right_delta - odom_left_delta));
  Serial.print("theta delta radian:  "); Serial.print(theta_delta); Serial.println(" ");

  yaw += theta_delta;
  theta_delta_degree = RAD2DEG(theta_delta);
  Serial.print("yaw radian:  "); Serial.print(yaw); Serial.println(" ");
  Serial.print("theta delta degree:  "); Serial.print(theta_delta_degree); Serial.println(" ");
  yaw_degree += theta_delta_degree;
  Serial.print("yaw degree:  "); Serial.print(yaw_degree); Serial.println(" ");
  
  /*
  Serial.print("delta_encoder_right:  "); Serial.print(delta_encoder_right); Serial.print("  ");
  Serial.print("delta_encoder_left:  ");  Serial.print(delta_encoder_left); Serial.println("  ");
  Serial.print("odom_right_delta:  "); Serial.print(odom_right_delta); Serial.print("     ");
  Serial.print("odom_left_delta:  "); Serial.print(odom_left_delta); Serial.println("    ");
  Serial.print("wheel_track:  "); Serial.print(wheel_track); Serial.println("     ");
  theta_delta_degree = theta_delta * 180.0/3.14150;
  Serial.print("theta_delta:  "); Serial.print(theta_delta_degree); Serial.println(" ");
  Serial.println();

  //Serial.print("cnt1_old:  "); Serial.print(cnt1_old); Serial.println("    ");
  //Serial.print("cnt2_old:  "); Serial.print(cnt2_old); Serial.println("    ");
  //Serial.println();

  Serial.print("pulse_to_distance_right:  "); Serial.print(pulse_to_distance_right); Serial.println("    ");
  Serial.print("pulse_to_distance_left:  "); Serial.print(pulse_to_distance_left); Serial.println("    ");
  Serial.println();
  */

  cnt1_old = cnt1;
  cnt2_old = cnt2;
}

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
  MsTimer2::set(100, msTimer2_ISR);
  MsTimer2::start();
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
  //Serial.print("cnt1: ");
  //Serial.print(cnt1);
  //Serial.print("       cnt2: ");
  //Serial.println(cnt2);
  //Serial.println();
}
