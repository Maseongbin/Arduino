
/////////////////////IMU///////////////////////
#include <Wire.h>
#include <LSM303.h>

#define THRESHOLD_ANGLE 18

LSM303 compass;

float heading_angle      = 0.0;
float init_heading_angle = 17.0; // 초기 방향
float target_heading_angle = 90;
float heading_angle_error = 0.0; // error 값
/////////////////////sonar///////////////////////

#include <NewPing.h>

#define SONAR_NUM 3      // Number of sensors.
#define MAX_DISTANCE 150 // Maximum distance (in cm) to ping.
#define WALL_GAP_DISTANCE      400//mm 단위
#define WALL_GAP_DISTANCE_HALF 150 //mm 단위
#define MOTOR_PWM_OFFSET 10


#define Front 0
#define Left  1 
#define Right 2

#define TRIG1 2 // 초음파 센서 1번 Trig 핀 번호
#define ECHO1 3 // 초음파 센서 1번 Echo 핀 번호

#define TRIG2 4 // 초음파 센서 2번 Trig 핀 번호
#define ECHO2 5 // 초음파 센서 2번 Echo 핀 번호

#define TRIG3 6 // 초음파 센서 3번 Trig 핀 번호
#define ECHO3 7 // 초음파 센서 3번 Echo 핀 번호

NewPing sonar[SONAR_NUM] = {   // Sensor object array.
NewPing(TRIG1, ECHO1, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
NewPing(TRIG2, ECHO2, MAX_DISTANCE),
NewPing(TRIG3, ECHO3, MAX_DISTANCE)
};

float front_sonar = 0.0;
float left_sonar  = 0.0;
float right_sonar = 0.0;

/////////////////////L298//////////////////////////
#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

 
/////////////////////Maze_Status//////////////////////////
int maze_status = 0; 


void setup() 
{

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);
  
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
   Serial.begin(115200); // 통신속도를 115200으로 정의함


  Wire.begin(); //IMU initiallize
  compass.init();
  compass.enableDefault();

   /*
  Calibration values; the default values of +/-32767 for each axis
  lead to an assumed magnetometer bias of 0. Use the Calibrate example
  program to determine appropriate values for your particular unit.
  */
  compass.m_min = (LSM303::vector<int16_t>){-32767, -32767, -32767};
  compass.m_max = (LSM303::vector<int16_t>){+32767, +32767, +32767};
}


void motor_R_control(int motor_speed_r) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if(motor_speed_r >= 0)
  {
     digitalWrite(IN3, HIGH); //모터의 방향 제어
     digitalWrite(IN4, LOW);
     if(motor_speed_r>=225) motor_speed_r = 225;
     analogWrite(ENB,motor_speed_r); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    if(motor_speed_r<=-225) motor_speed_r = -225;
    analogWrite(ENB,-motor_speed_r);
    
  }
}

void motor_L_control(int motor_speed_l) //모터 A의 방향(direction)과 속도(speed) 제어
{
  if(motor_speed_l >= 0)
  {
     digitalWrite(IN1, LOW); //모터의 방향 제어
     digitalWrite(IN2, HIGH);
     if(motor_speed_l>=225) motor_speed_l = 225;
     analogWrite(ENA,motor_speed_l); //모터의 속도 제어
    
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    if(motor_speed_l<=-225) motor_speed_l = -225;
    analogWrite(ENA,-motor_speed_l);
    
  }
}


void check_maze_status(void)
{
  if((left_sonar>=0) && (left_sonar<=WALL_GAP_DISTANCE) && (right_sonar>=0) && (right_sonar<=WALL_GAP_DISTANCE) && (front_sonar>=0) && (front_sonar<=WALL_GAP_DISTANCE_HALF)) // 3면이 다 막힌 경우
  {
    maze_status = 4;
    Serial.println("maze_status = 4");
  }
  else if( (left_sonar>=0) && (left_sonar<=WALL_GAP_DISTANCE) && (right_sonar>=0) && (right_sonar<=WALL_GAP_DISTANCE) && (front_sonar>=WALL_GAP_DISTANCE_HALF)  )
  {
    maze_status = 1;
    Serial.println("maze_status = 1");
  }
  else if((left_sonar>=0) && (left_sonar<=WALL_GAP_DISTANCE) && (front_sonar>=0) && (front_sonar<=WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 2;
    Serial.println("maze_status = 2");
  }
  else if((right_sonar>=0) && (right_sonar<=WALL_GAP_DISTANCE) && (front_sonar>=0) && (front_sonar<=WALL_GAP_DISTANCE_HALF))
  {
    maze_status = 3;
    Serial.println("maze_status = 3");
  }
  else
  {
    maze_status = 0;
    Serial.println("maze_status = 0"); 
  }
}
//먼저 left_sonar = 0; right_sonar = 0; 으로 해서 오른쪽 방향 찾기
void wall_collision_avoid(int base_spped)
{
  float error = 0.0;
  float Kp = 0.7; //나중에 조정해야 할 값(얼마나 돌지)
  int pwm_control = 0;
  int right_pwm = 0;
  int left_pwm  = 0;
  error = (right_sonar - left_sonar);
  error = Kp * error;  
  
  if(error >= 50) error = 50;
  if(error <= -50) error = -50; 
                       

  right_pwm = base_spped - error;
  left_pwm  = base_spped + error;
  
  if(right_pwm <= 0) right_pwm = 0;
  if(left_pwm  <= 0) left_pwm  = 0;

  if(right_pwm >= 150) right_pwm = 150;
  if(left_pwm  >= 70) left_pwm  = 70;

  
}

void imu_rotation(void)
{
  compass.read();
  float heading1 = compass.heading();
  compass.read();
  float heading2 = compass.heading();
  float heading = (heading1+heading2)/2;

   heading = 360 - heading; // 회전 좌표계를 반시계 방향으로 할 것
  
  Serial.println(heading); 

  heading_angle_error = target_heading_angle - heading;  
  
  if(heading_angle_error > 180)
  {
    heading_angle_error = heading_angle_error - 360;
  }
  else if(heading_angle_error < -180)
  {
    heading_angle_error = heading_angle_error + 360;
  }
  else
  {
    
  }
  Serial.print("Heading Angle Error: ");
  Serial.print(heading_angle_error); //heading_angle_error 표시
  Serial.print(" = ");
  Serial.print(target_heading_angle);
  Serial.print(" - ");
  Serial.println(heading); //heading angle 표시

  
   
  if(heading_angle_error > THRESHOLD_ANGLE) // 반시계방향으로 회전
  {
    motor_R_control(80); motor_L_control(-80); 
  }
  else if((heading_angle_error >= -THRESHOLD_ANGLE) && (heading_angle_error <= THRESHOLD_ANGLE)) // 정지
  {
    motor_R_control(0); //왼쪽 바퀴가 앞으로 도는 지 확인
    motor_L_control(0);
  }
  else // heading_angle_error <= THRESHOLD_ANGLE  //시계방향으로 회전
  {
    motor_R_control(-80); motor_L_control(80); 
  }
}
void loop() 
{
  //Serial.print("Duration: ");
  //Serial.println(duration);
 
  float front_sonar1 = sonar[Front].ping_cm()*10; // 전방 센서 측정
  float left_sonar1  = sonar[Left].ping_cm() *10; // 좌축 센서 측정
  float right_sonar1 = sonar[Right].ping_cm()*10; // 우측 센서 측정

  float front_sonar2 = sonar[Front].ping_cm()*10; // 전방 센서 측정
  float left_sonar2  = sonar[Left].ping_cm() *10; // 좌축 센서 측정
  float right_sonar2 = sonar[Right].ping_cm()*10; // 우측 센서 측정

  float front_sonar3 = sonar[Front].ping_cm()*10; // 전방 센서 측정
  float left_sonar3  = sonar[Left].ping_cm() *10; // 좌축 센서 측정
  float right_sonar3 = sonar[Right].ping_cm()*10; // 우측 센서 측정

  front_sonar = (front_sonar1 + front_sonar2 + front_sonar3) / 3;
  left_sonar  = (left_sonar1 + left_sonar2 + left_sonar3) / 3;
  right_sonar = (right_sonar1 + right_sonar2 + right_sonar3) / 3;
  
  if(front_sonar == 0.0) front_sonar = MAX_DISTANCE*10; // 0.0 출력이 최대값이므로
  if(left_sonar  == 0.0)  left_sonar = MAX_DISTANCE*10;
  if(right_sonar == 0.0) right_sonar = MAX_DISTANCE*10;

  Serial.print("L: "); Serial.print(left_sonar); Serial.print(" ");
  Serial.print("F: "); Serial.print(front_sonar); Serial.print(" ");
  Serial.print("R: "); Serial.println(right_sonar);

  

  
 

  check_maze_status();
  
  imu_rotation();
  
/*
  if(maze_status == 4)
  {
  //정지 한다
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  
  
  //180도 회전을 한다
  Serial.println("Rotate CCW");
  motor_A_control(HIGH,85); //왼쪽 전진
  motor_B_control(LOW,90); //오른쪽은 후진
  delay(1500);
  
  //정지 한다
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  
  
  //앞으로 간다
  Serial.println("Go Straight");
  motor_A_control(HIGH,100);
  motor_B_control(HIGH,110);

  }
  
  if(maze_status == 1)
  {
  //앞으로 간다
  Serial.println("run straight");
  wall_collision_avoid(100);
  } 
  
  else if(maze_status == 3)
  {
  //정지 한다
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  
  //왼쪽으로 90도 회전 한다
  Serial.println("Rotate CCW");
  motor_A_control(LOW,95); 
  motor_B_control(HIGH,100); 
  delay(700);
  //정지 한다 
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
   
  }
  
  else if(maze_status == 2)
  {
  //정지 한다
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  
  //오른쪽으로 90도 회전 한다
  Serial.println("Rotate CCW");
  motor_A_control(HIGH,90); 
  motor_B_control(LOW,95); 
  delay(700);
  //정지 한다
  motor_A_control(HIGH,0);
  motor_B_control(LOW,0);
  
  } 
  
  else
  {
   //앞으로 간다
  Serial.println("Go Straight");
  motor_A_control(HIGH,100);
  motor_B_control(HIGH,110);
  }
*/
 }
