#include <NewPing.h>
#include <Servo.h>
#include <MsTimer2.h>

#define LINE_PRINT   1
#define encodPinA1   2
#define encodPinB1   3
#define MOTOR_DIR    4
#define MOTOR_PWM    5
#define A0pin        A0
#define SIpin        13
#define CLKpin       12
#define RC_SERVO_PIN 11

#define LEFT_STEER_ANGLE    -45
#define RIGHT_STEER_ANGLE    45
#define NEURAL_ANGLE         90
#define NEURAL_ANGLE_offset   -1

#define MAX_DISTANCE  350

#define threshold_valus 60
#define NPIXELS         128
#define OFFSET          0

float Kp = 1.0;
float Ki = 0;
float Kd = 1.2;

byte Pixel[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];   // 최대최소 구분
int MAX_LineSensor_Data[NPIXELS];       // Max value of sensor
int MIN_LineSensor_Data[NPIXELS];       // Min value of sensor

int Steering_Angle = 0;
int steer_data = 0;

int Line_Center       = NPIXELS / 2;
int Line_L_Center     = 10;
int Line_R_Center     = NPIXELS - 10;

int Line_L_Center_old = 10;
int Line_R_Center_old = NPIXELS - 10;


float error    = 0;
float error_s  = 0;
float error_d  = 0;
float error_old = 0;

//double Line_L_Center;
//double Line_R_Center;

/*
  ////////////////////////////// SONAR ////////////////////////////////////////////
  NewPing R_sensor(49, 48, MAX_DISTANCE);
  float R_Sonar_Error = 0.0;
  float R_Sonar_distance = 0.0;
  float R_Sonar_distance_old = 0.0;

  NewPing F_sensor(53, 52, MAX_DISTANCE);
  float L_Sonar_Error = 0.0;
  float L_Sonar_distance = 0.0;
  float L_Sonar_distance_old = 0.0;

  NewPing L_sensor(51, 50, MAX_DISTANCE);
  float F_Sonar_Error = 0.0;
  float F_Sonar_distance = 0.0;
  float F_Sonar_distance_old = 0.0;


  void read_sonar_sensor(void)
  { //초음파센서 측정
  R_Sonar_distance = R_sensor.ping_cm() * 10.0;
  L_Sonar_distance = L_sensor.ping_cm() * 10.0;
  F_Sonar_distance = F_sensor.ping_cm() * 10.0;
  if (R_Sonar_distance == 0)
  {
    R_Sonar_distance = MaxDistance * 10.0;
  }
  if (L_Sonar_distance == 0)
  {
    L_Sonar_distance = MaxDistance * 10.0;
  }
  if (F_Sonar_distance == 0)
  {
    F_Sonar_distance = MaxDistance * 10.0;
  }
  }

  void update_sonar_old(void)
  { //초음파 센서의 옛날값 저장
  R_Sonar_distance_old = R_Sonar_distance;
  L_Sonar_distance_old = L_Sonar_distance;
  F_Sonar_distance_old = F_Sonar_distance;
  }

  void update_sonar_error(void)
  { //초음파 센서의 옛날값과 현재값의 차이 저장
  R_Sonar_Error = R_Sonar_distance - R_Sonar_distance_old;
  L_Sonar_Error = L_Sonar_distance - L_Sonar_distance_old;
  F_Sonar_Error = F_Sonar_distance - F_Sonar_distance_old;
  }


  void read_ultrasonic_sensor(void)
  {
  UltrasonicSensorData[0] = sonar[0].ping_cm();
  UltrasonicSensorData[1] = sonar[1].ping_cm();
  UltrasonicSensorData[2] = sonar[2].ping_cm();
  UltrasonicSensorData[3] = sonar[3].ping_cm();
  UltrasonicSensorData[4] = sonar[4].ping_cm();
  }

  void Robot_Mode_Define(void)
  {
  read_ultrasonic_sensor();
  mode = 0;
  for (int i = 0; i < 4; i++)
  {
    if (UltrasonicSensorData[i] == 0) UltrasonicSensorData[i] = MAX_DISTANCE;
  }

  //양 옆에 벽이 있고, 앞에 가로막음
  if ((UltrasonicSensorData[0] <= 20) &&  (UltrasonicSensorData[1] <= 20) && (UltrasonicSensorData[2] <= 25))
    mode = 1;
  }
*/

volatile long encoderPos = 0;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

Servo Steeringservo;


int steering_control()
{
  // 서보 angle값 제한
  if (Steering_Angle <= LEFT_STEER_ANGLE)  Steering_Angle = LEFT_STEER_ANGLE;
  if (Steering_Angle >= RIGHT_STEER_ANGLE) Steering_Angle = RIGHT_STEER_ANGLE;

  Steeringservo.write(Steering_Angle + NEURAL_ANGLE + NEURAL_ANGLE_offset);
}

void encoderB()
{
  delayMicroseconds(2);
  if (digitalRead(encodPinB1) == LOW) encoderPos++;
  else                                encoderPos--;
}

/*
  void line_adaptation(void)
  {
  //
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (LineSensor_Data[i] >= MAX_LineSensor_Data[i]) MAX_LineSensor_Data[i] = LineSensor_Data[i];
    if (LineSensor_Data[i] <= MIN_LineSensor_Data[i]) MIN_LineSensor_Data[i] = LineSensor_Data[i];
  }
  }
*/

void read_line_sensor(void)
{
  int i;
  delayMicroseconds (1);
  delay(10);

  digitalWrite (CLKpin, LOW);
  digitalWrite (SIpin, HIGH);
  digitalWrite (CLKpin, HIGH);
  digitalWrite (SIpin, LOW);

  delayMicroseconds (1);

  for (i = 0; i < NPIXELS; i++)
  {
    Pixel[i] = analogRead (A0pin);
    digitalWrite (CLKpin, LOW);
    delayMicroseconds (1);
    digitalWrite (CLKpin, HIGH);
  }

}

void line_data_serial_print(void)
{
  if (LINE_PRINT != 1)  return;

  for (int i = 0; i < NPIXELS; i++)
  {
    Serial.print(LineSensor_Data_Adaption[i]);
    //Serial.print((byte)Pixel[i]);
    Serial.print(" ");
  }

  Serial.print(Line_L_Center); //노랑
  Serial.print(" ");
  Serial.print(Line_R_Center); //파랑
  Serial.print(" ");
  Serial.print(Line_Center);   //빨강
  Serial.println(" ");
}

void threshold(void)
{
  int i;
  for (i = 0; i < NPIXELS; i++)
  {
    if (Pixel[i] >= threshold_valus)
    {
      LineSensor_Data_Adaption[i] = 255;
    }
    else
    {
      LineSensor_Data_Adaption[i] = 0;
    }
  }
}

void find_line_center(void)
{
  int i;
  long sum = 0;
  long x_sum = 0;
  int distance_L = 0; int distance_R = 0;

  for (i = 0; i < NPIXELS; i++)
  {
    sum += LineSensor_Data_Adaption[i];
    x_sum += (LineSensor_Data_Adaption[i]) * i;
  }
  Line_Center = (x_sum / sum);

  if (LineSensor_Data_Adaption[Line_Center] != 255) // Line이 2개 검출 될때
  {
    for (i = 0, sum = 0, x_sum = 0; i < Line_Center; i++)
    {
      sum  += LineSensor_Data_Adaption[i];
      x_sum += LineSensor_Data_Adaption[i] * i;
    }

    if (sum !=  0) Line_L_Center = (x_sum / sum);

    for (i = Line_Center, sum = 0, x_sum = 0; i < NPIXELS; i++)
    {
      sum += LineSensor_Data_Adaption[i];
      x_sum += LineSensor_Data_Adaption[i] * i;
    }

    if (sum !=  0) Line_R_Center = (x_sum / sum);

    Line_Center = (Line_R_Center + Line_L_Center) / 2;
  }
  else
  {
    distance_L = abs(Line_Center - Line_L_Center_old);
    distance_R = abs(Line_Center - Line_R_Center_old);
    if (distance_L < distance_R)
    {
      Line_L_Center = Line_Center;
      Line_R_Center = Line_L_Center + 33;
    }

    if (distance_L > distance_R)
    {
      Line_R_Center = Line_Center;
      Line_L_Center = Line_R_Center - 33;
    }

    Line_Center = (Line_R_Center + Line_L_Center) / 2;
  }

  Line_L_Center_old = Line_L_Center;
  Line_R_Center_old = Line_R_Center;

}

void PID_line_control()
{

  error   = Line_Center - NPIXELS / 2 + OFFSET ; // 카메라 보정 방향 반대시 전체에 -1 곱하기
  error_d = error - error_old;
  error_s = (error_s >= 5) ?   5 : error_s;
  error_s = (error_s <= -5) ? -5 : error_s;

  Steering_Angle = Kp * error + Kd * error_d + Ki * error_s;

  if (fabs(error) <= 1)
  {
    error_s = 0;
  }

  steering_control();
  Serial.print(Line_Center);
  Serial.print("   ");
  Serial.print("error: ");
  Serial.print(error);
  Serial.print("   ");
  Serial.print("error_old: ");
  Serial.print(error_old);
  Serial.print("   ");
  Serial.print("error_d: ");
  Serial.print(error_d);
  Serial.print("   ");
  Serial.print("Kp * error: ");
  Serial.print(Kp * error);
  Serial.print("   ");
  Serial.print("Kd * error_d: ");
  Serial.print(Kd * error_d);
  Serial.print("   ");
  Serial.print(Steering_Angle);
  Serial.println("   ");

  error_old = error;

}

void motor_control(int speed)
{
  if (speed >= 0) {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, speed);
  }
  else {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, -speed);
  }
}

void CallBack()
{
  read_line_sensor();
  threshold();
  //line_data_serial_print();
  find_line_center();
  PID_line_control();
}

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);                  // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);                  // quadrature encoder input B
  attachInterrupt(0, encoderB, FALLING);               // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;                   // To prevent Motor Noise
}

void setup() {
  // put your setup code here, to run once:
  for (int a = 0; a < NPIXELS; a++) {
    LineSensor_Data[a] = 0;
    LineSensor_Data_Adaption[a] = 0;
    MAX_LineSensor_Data[a] = 1023;
    MIN_LineSensor_Data[a] = 0;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);
  // pinMode(A0pin, INPUT);
  digitalWrite(CLKpin, LOW);
  digitalWrite(SIpin, LOW);

#if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
#endif

  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);

  Steeringservo.attach(RC_SERVO_PIN);
  Steeringservo.write(NEURAL_ANGLE);

  interrupt_setup();

  MsTimer2::set(50, CallBack);
  MsTimer2::start();

  Serial.begin(115200);

  //delay(500);
}

void loop()
{
  /*
    Robot_Mode_Define();
    switch (mode)
    {
    case 0:
      motor_control(150); //속도 130
      break;
    case 1:
      motor_control(0); //속도 130
      break;
    default:
      motor_control(150);
      break;
    }
  */
  // Steeringservo.write(NEURAL_ANGLE + NEURAL_ANGLE_offset);
  motor_control(150); //속도 130
}
