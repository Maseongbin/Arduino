#include <Wire.h>
#include <Servo.h>

#define SLAVE_ADDRESS 0x05

#define NEUTRAL_ANGLE 90
#define RC_SERVO_PIN   8

#define encodPinA1   2
#define encodPinB1   3

#define MOTOR_DIR    4
#define MOTOR_PWM    5

#define FASTADC 1
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

int Steering_Angle = 0;
Servo Steeringservo;

union 
{
  short data;
  byte bytes[2];
} car_speed, car_servo;

union
{
  int encoder_data;
  byte encoder_bytes[2];
}encoderpos;

void setup() 
{
  #if FASTADC
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  cbi(ADCSRA, ADPS0);
  
  #endif
  pinMode(MOTOR_DIR, OUTPUT);
  pinMode(MOTOR_PWM, OUTPUT);
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(requestData);
  
  interrupt_setup();
  
  Serial.begin(115200);
  Steeringservo.attach(RC_SERVO_PIN);   
}

void motor_control(int speed)
{
  if (speed >= 0) 
  {
    digitalWrite(MOTOR_DIR, LOW);
    analogWrite(MOTOR_PWM, speed);
  }
  else 
  {
    digitalWrite(MOTOR_DIR, HIGH);
    analogWrite(MOTOR_PWM, -speed);
  }
  //Serial.print("speed: ");
  //Serial.println(speed);
  //Serial.println();    
}

unsigned char encoderpos_data[6] = {'D', 0, 0, 0, 0, '*'};
volatile long encoderPos = 0;

void encoderB()
{
  delayMicroseconds(2);
  if (digitalRead(encodPinB1) == HIGH) 
  {
    encoderPos++;
  }
  else 
  {
    encoderPos--;
  }
}

void interrupt_setup(void)
{
  pinMode(encodPinA1, INPUT_PULLUP);        // quadrature encoder input A
  pinMode(encodPinB1, INPUT_PULLUP);        // quadrature encoder input B
  attachInterrupt(0, encoderB, FALLING);    // update encoder position
  TCCR1B = TCCR1B & 0b11111000 | 1;         // To prevent Motor Noise
}

void encoder_serial_print(void)
{
  Serial.print("encoderpos: ");
  Serial.println(encoderPos);
  //delay(1000);
}

void loop() 
{
  encoder_serial_print();
  delay(100);
}

void receiveData(int byteCount) 
{
  if (Wire.available() >= 9) 
  {
    byte receivedData[9];
    for (int i = 0; i < 9; i++) 
    {
      receivedData[i] = Wire.read(); 
    }

    if (receivedData[0] == '#' && receivedData[1] == 'C' && receivedData[8] == '*') 
    {
      car_servo.bytes[0] = receivedData[2];
      car_servo.bytes[1] = receivedData[3];
      short angle = car_servo.data;

      car_speed.bytes[0] = receivedData[4];
      car_speed.bytes[1] = receivedData[5];
      float speed = car_speed.data;
      
      Steeringservo.write(NEUTRAL_ANGLE + angle);
  
      //Serial.print("angle_offset: ");
      //Serial.println(angle);
      //Serial.print("angle: ");
      //Serial.println(NEUTRAL_ANGLE + angle);
      //Serial.println();
      //delay(1000);
      
      motor_control(speed);
    } 
    else 
    {
      Serial.println("Invalid protocol");
    }
  }
}

void requestData(int byteCount)
{
  encoderpos.encoder_data = encoderPos;
  encoderpos_data[0] = 'D';
  encoderpos_data[1] = encoderpos.encoder_bytes[0];
  encoderpos_data[2] = encoderpos.encoder_bytes[1];
  encoderpos_data[3] = 0;
  encoderpos_data[4] = 0;
  encoderpos_data[5] = '*';
  Wire.write(encoderpos_data, sizeof(encoderpos_data));
}
