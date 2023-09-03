#define ENA 8
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13

void setup() 
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  Serial.begin(115200);
}

void motor_R_control(int motor_speed_r)         //모터 A의 방향(direction)과 속도(speed)제어
{  
  if (motor_speed_r >= 0)
    {
      digitalWrite(IN1, LOW);         //모터의 방향 제어
      digitalWrite(IN2, HIGH);
      if(motor_speed_r>=255) motor_speed_r = 255;
      analogWrite(ENB, motor_speed_r); //모터의 속도 제어
    }
    else
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      if(motor_speed_r<=-255) motor_speed_r = -255;
      analogWrite(ENB, -motor_speed_r);
    }
}

void motor_L_control(int motor_speed_l)     //모터 B의 방향(direction)과 속도(speed)제어
{  
    if (motor_speed_l >= 0)
    {
      digitalWrite(IN3, LOW);         //모터의 방향 제어
      digitalWrite(IN4, HIGH);
      if(motor_speed_l>=255) motor_speed_l = 255;
      analogWrite(ENA, motor_speed_l); //모터의 속도 제어
    }
    else
    {
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      if(motor_speed_l<=-255) motor_speed_l = -255;
      analogWrite(ENA, -motor_speed_l);
    }
}

void loop() 
{
  int motor_speed_R = 50;
  int motor_speed_L = 50;
  
  motor_R_control(motor_speed_R);
  motor_L_control(motor_speed_L);

}
