#include <MsTimer2.h>

#define LINE_PRINT   1

#define pulse 232.0
#define pulse_m 1160.0
double  pulse_distance;

#define encodPinA1   2
#define encodPinB1   3
#define MOTOR_DIR    4
#define MOTOR_PWM    5

#include <Servo.h>
#define LEFT_STEER_ANGLE    -50
#define RIGHT_STEER_ANGLE    50
#define NEURAL_ANGLE         90
#define NEURAL_ANGLE_offset  -3

int Steering_Angle = 0;

Servo Steeringservo;

#define A0pin        A0
#define SIpin        13
#define CLKpin       12
#define RC_SERVO_PIN 11

#define threshold_valus  75
#define NPIXELS         128
#define OFFSET         -1.5

byte Pixel[NPIXELS];

int LineSensor_Data[NPIXELS];
int LineSensor_Data_Adaption[NPIXELS];  
int MAX_LineSensor_Data[NPIXELS];     
int MIN_LineSensor_Data[NPIXELS];     

int Line_Center       = NPIXELS / 2;
int Line_L_Center     = 10;
int Line_R_Center     = NPIXELS - 10;

int Line_L_Center_old = 10;
int Line_R_Center_old = NPIXELS - 10;

#include <NewPing.h>
#define MAX_DISTANCE 1500

float UltrasonicSensorData[3];
float sonar_data = 0;

float Kp = 1.2;
float Ki = 0;
float Kd = 1.4;

float Kp_curve = 1.5;
float Ki_curve = 0;
float Kd_curve = 1.15;

float Kp_maze  = 0.09; 
float Ki_maze  = 0;
float Kd_maze  = 0.1;
float Pi = 0;

float error    = 0;
float error_s  = 0;
float error_d  = 0;
float error_old = 0;

float error_maze = 0;
float error_maze_old = 0;

float error_curve = 0;
float error_curve_old = 0;
float error_curve_d  = 0;

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

int mode = 1;
int mission_flag = 0;

void setup()
{
  for (int a = 0; a < NPIXELS; a++)
  {
    LineSensor_Data[a] = 0;
    LineSensor_Data_Adaption[a] = 0;
    MAX_LineSensor_Data[a] = 1023;
    MIN_LineSensor_Data[a] = 0;
  }

  pinMode(SIpin, OUTPUT);
  pinMode(CLKpin, OUTPUT);

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

  Serial.begin(115200);
}

void motor_control(int speed)
{
  if (speed >= 0)
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, speed);
  }
  else 
  {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, -speed);
  }
}

int steering_control()
{
  if (Steering_Angle <= LEFT_STEER_ANGLE)  Steering_Angle = LEFT_STEER_ANGLE;
  if (Steering_Angle >= RIGHT_STEER_ANGLE) Steering_Angle = RIGHT_STEER_ANGLE;

  Steeringservo.write(Steering_Angle + NEURAL_ANGLE + NEURAL_ANGLE_offset);
}

void steering_turn_right(void)
{
  Steeringservo.write(29 + NEURAL_ANGLE + NEURAL_ANGLE_offset);  //37.5
}

void steering_turn_right_s(void)
{
  Steeringservo.write(28.3 + NEURAL_ANGLE + NEURAL_ANGLE_offset);
}

void steering_turn_right_obs(void)
{
  Steeringservo.write(42 + NEURAL_ANGLE + NEURAL_ANGLE_offset);
}

void steering_turn_left(void)
{
  Steeringservo.write(-43 + NEURAL_ANGLE + NEURAL_ANGLE_offset);
}

void steering_0(void)
{
  Steeringservo.write(NEURAL_ANGLE + NEURAL_ANGLE_offset);
}

volatile long encoderPos = 0;

void encoderB()
{
  delayMicroseconds(2);
  if (digitalRead(encodPinB1) == LOW) encoderPos++;
  else                                encoderPos--;
}

void reset_encoder(void)
{
  encoderPos = 0;
}

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);        
  pinMode(encodPinB1, INPUT_PULLUP);        
  attachInterrupt(0, encoderB, FALLING);    
  TCCR1B = TCCR1B & 0b11111000 | 1;        
}

void encoder_serial_print(void)
{
  Serial.print("encoderPos: ");
  Serial.println(encoderPos);
}

void pulse_conversion(void)
{
  encoderB();
  pulse_distance = encoderPos / pulse_m;
}

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
    Serial.println((byte)Pixel[i]);
    Serial.print(" ");
  }

  Serial.print(Line_L_Center); 
  Serial.print(" ");
  Serial.print(Line_R_Center); 
  Serial.print(" ");
  Serial.print(Line_Center);  
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

  if (LineSensor_Data_Adaption[Line_Center] != 255)
  {
    for (i = 0, sum = 0, x_sum = 0; i < Line_Center; i++)
    {
      sum  += LineSensor_Data_Adaption[i];
      x_sum += LineSensor_Data_Adaption[i] * i;
    }

    if (sum !=  0)
    {
      Line_L_Center = (x_sum / sum);
    }

    for (i = Line_Center, sum = 0, x_sum = 0; i < NPIXELS; i++)
    {
      sum += LineSensor_Data_Adaption[i];
      x_sum += LineSensor_Data_Adaption[i] * i;
    }

    if (sum !=  0)
    {
      Line_R_Center = (x_sum / sum);
    }

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

NewPing sonar[3] =
{
  NewPing(53, 52, MAX_DISTANCE),
  NewPing(51, 50, MAX_DISTANCE),
  NewPing(49, 48, MAX_DISTANCE)
};

void read_ultrasonic_sensor(void)
{
  UltrasonicSensorData[0] = sonar[0].ping_cm() * 10.0; 
  UltrasonicSensorData[1] = sonar[1].ping_cm() * 10.0; 
  UltrasonicSensorData[2] = sonar[2].ping_cm() * 10.0;  

  if (  UltrasonicSensorData[0] == 0)
  {
    UltrasonicSensorData[0] = MAX_DISTANCE;
  }
}

void Robot_Mode_Define(void)
{
  for (int i = 0; i < 3; i++)
  {
    if (UltrasonicSensorData[i] == 0)
    {
      UltrasonicSensorData[i] = MAX_DISTANCE;
    }
  }

  if ((UltrasonicSensorData[1] <= 600) && (UltrasonicSensorData[2] <= 600))
  {
    mode = 1;
  }

  if ((UltrasonicSensorData[1] <= 600) && (UltrasonicSensorData[2] >= 600) && (UltrasonicSensorData[0] <= 950))
  {
    mode = 2;
  }
}

void ultrasonic_sensor_serial_print(void)
{
  read_ultrasonic_sensor();

  Serial.print("F_sonar: ");
  Serial.print(UltrasonicSensorData[0]);
  Serial.print(" ");

  Serial.print("L_sonar: ");
  Serial.print(UltrasonicSensorData[1]);
  Serial.print(" ");

  Serial.print("R_sonar: ");
  Serial.print(UltrasonicSensorData[2]);
  Serial.println(" ");
}

void read_front_sonar_sensor(void)
{
  UltrasonicSensorData[0] = sonar[0].ping_cm() * 10.0;  // 전방
}

void PID_line_control()
{
  error   = Line_Center - NPIXELS / 2 + OFFSET ;
  error_d = error - error_old;
  error_s = (error_s >= 5) ?   5 : error_s;
  error_s = (error_s <= -5) ? -5 : error_s;

  Steering_Angle = Kp * error + Kd * error_d + Ki * error_s;

  if (fabs(error) <= 1)
  {
    error_s = 0;
  }

  steering_control();
  
  error_old = error;
}

void PID_curve_control()
{
  error_curve   = Line_Center - NPIXELS / 2 + OFFSET ;
  error_curve_d = error_curve - error_curve_old;

  Steering_Angle = Kp_curve * error_curve + Kd_curve * error_d;

  steering_control();

  error_curve_old = error_curve;
}

float PID_Control_wall(float error)
{
  float PIDvalue = 0;
  float error_maze_d = 0;
  error_maze = error;
  Pi = Pi + error_maze;
  error_maze_d = error_maze - error_maze_old;

  PIDvalue = (Kp_maze * error_maze) + (Kd_maze * error_maze_d);

  error_maze_old = error_maze;

  return PIDvalue;
}

void wall_following(void)
{
  sonar_data = UltrasonicSensorData[2] - UltrasonicSensorData[1];
  Steering_Angle = PID_Control_wall(sonar_data);

  steering_control();
}

void CallBack()
{
  read_line_sensor();
  threshold();
  line_data_serial_print();
  find_line_center();
  PID_line_control();
}

void lane_control(void)
{
  read_line_sensor();
  threshold();
  line_data_serial_print();
  find_line_center();
  PID_line_control();
}

void lane_curve_control(void)
{
  read_line_sensor();
  threshold();
  line_data_serial_print();
  find_line_center();
  PID_curve_control();
}

int timer = 0;

void Obstacle(void)
{
  steering_turn_left();

  if (encoderPos < 600)
  {
    motor_control(150);
  }
  if ((encoderPos >= 600) && (encoderPos < 1100))
  {
    steering_turn_right_obs();
    motor_control(150);
  }
  if (encoderPos >= 1100)
  {
    reset_encoder();
    mission_flag = 8;
  }
}

void loop()
{
  switch (mission_flag)
  {
    case 0:

      read_front_sonar_sensor();

      if (UltrasonicSensorData[0] <= 300)
      {
        motor_control(0);
      }
      else
      {
        reset_encoder();
        mission_flag = 1;
      }

      break;

    case 1:

      lane_control();

      if (encoderPos < 3600)
      {
        motor_control(250);
      }

      if ( (encoderPos >= 3600) && (encoderPos < 6700))
      {
        motor_control(180);
      }
      if ( (encoderPos >= 6700) && (encoderPos < 10000))
      {
        motor_control(230);
      }
      if ( (encoderPos >= 10000) && (encoderPos < 12000))
      {
        motor_control(205);
      }
      if (encoderPos >= 12000)
      {
        motor_control(0);
        reset_encoder();
        delay(10);
        mission_flag = 2;
      }
      break;
      
    case 2:

      read_ultrasonic_sensor();
      wall_following();
      motor_control(210);

      if (encoderPos >= 1300)
      {
        motor_control(0);
        delay(10);
        reset_encoder();

        mission_flag = 3;
      }

      break;

    case 3:

      read_ultrasonic_sensor();
      steering_turn_right();
      motor_control(180);

      if (encoderPos >= 800)
      {
        motor_control(0);
        steering_0();
        reset_encoder();

        delay(10);

        mission_flag = 4;
      }

      break;

    case 4:

      read_ultrasonic_sensor();
      wall_following();
      motor_control(200);

      if (encoderPos >= 4100)
      {
        reset_encoder();
        delay(10);
        mission_flag = 5;
      }

      break;

    case 5:

      steering_turn_right_s();
      motor_control(180);

      if (encoderPos >= 1360)
      {
        steering_0();
        reset_encoder();
        motor_control(0);
        delay(10);
        mission_flag = 6;
      }
      break;

    case 6:

      if (encoderPos < 4000)
      {
        lane_control();
        motor_control(180);
      }

      if ((encoderPos >= 4000) && (encoderPos < 9600 ))
      {
        lane_curve_control();
        motor_control(180);
      }
      if (encoderPos >= 9600)
      {
        motor_control(0);
        reset_encoder();
        delay(500);
        mission_flag = 7;
      }

      break;

    case 7:

      if (encoderPos < 700)
      {
        steering_turn_left();
        motor_control(150);
      }
      if ((encoderPos >= 700) && (encoderPos < 1400))
      {
        steering_turn_right_obs();
        motor_control(150);
      }
      if (encoderPos >= 1400)
      {
        reset_encoder();
        mission_flag = 8;
      }

      break;

    case 8:

      if (encoderPos < 7000)
      {
        lane_curve_control();
        motor_control(180);
      }

      if ((encoderPos >= 7000) && (encoderPos < 9150))
      {
        lane_control();
        motor_control(180);
      }

      if (encoderPos > 9150)
      {
        motor_control(0);
      }
      break;
  }
}
