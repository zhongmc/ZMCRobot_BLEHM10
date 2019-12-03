#include "ZMCRobot.h"

#define LEFT_WHEEL_DIR 4
#define LEFT_WHEEL_PWM 5

#define RIGHT_WHEEL_DIR 7
#define RIGHT_WHEEL_PWM 6

//read in whell dir
#define LEFT_WHEEL_A 3
#define LEFT_WHEEL_B 9 //12
#define RIGHT_WHEEL_A 2
#define RIGHT_WHEEL_B 8 //13  2 8

unsigned U0L = 0, U0R = 0;

volatile long count1 = 0;
volatile long count2 = 0;

void stopAndReset()
{
  StopMotor();
  //  lastError = 0;
  //  iTerm = 0;
  //  targetPosition = getWheelsPosition();
  //  lastRestAngle = targetAngle;
}

void initMotor()
{
  Serial.println("init motor...");
  log("LEFT pwm: %d dir: %d wa:%d wb:%d;\n", LEFT_WHEEL_PWM, LEFT_WHEEL_DIR, LEFT_WHEEL_A, LEFT_WHEEL_B );
  log("RIGHT pwm: %d dir: %d wa:%d wb:%d;\n", RIGHT_WHEEL_PWM, RIGHT_WHEEL_DIR, RIGHT_WHEEL_A, RIGHT_WHEEL_B );

  Serial.print("dtoi(2):" );
  Serial.println(digitalPinToInterrupt(RIGHT_WHEEL_A));
  Serial.print("dtoi(3):" );
  Serial.println(digitalPinToInterrupt(LEFT_WHEEL_A));


  pinMode(LEFT_WHEEL_A, INPUT_PULLUP);
  pinMode(RIGHT_WHEEL_A, INPUT_PULLUP);

  //改为change 让精度增加一倍
  attachInterrupt(digitalPinToInterrupt(LEFT_WHEEL_A), Code1, FALLING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_WHEEL_A), Code2, FALLING);

  // pinMode(LEFT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_WHEEL_DIR, OUTPUT);
  pinMode(LEFT_WHEEL_PWM, OUTPUT);

  //  pinMode(RIGHT_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_WHEEL_DIR, OUTPUT);
  pinMode(RIGHT_WHEEL_PWM, OUTPUT);

  //moto dir
  pinMode(LEFT_WHEEL_B, INPUT);
  pinMode(RIGHT_WHEEL_B, INPUT);

  digitalWrite(LEFT_WHEEL_DIR, HIGH);
  digitalWrite(RIGHT_WHEEL_DIR, HIGH);

  analogWrite(LEFT_WHEEL_PWM, 0);
  analogWrite(RIGHT_WHEEL_PWM, 0);
  count1 = 0;
  count2 = 0;
}

void StopMotor()
{
  //    digitalWrite(RIGHT_WHEEL_ENABLE, HIGH);
  //    digitalWrite(LEFT_WHEEL_ENABLE, HIGH);
  analogWrite(LEFT_WHEEL_PWM, 0);
  analogWrite(RIGHT_WHEEL_PWM, 0);
}

void MoveMotor(int pwm)
{

  MoveLeftMotor(pwm);
  MoveRightMotor(pwm);
}

void MoveLeftMotor(int PWM)
{
  int pwm_out;
  if (PWM >= 0)
  {
    pwm_out = PWM + U0L;
    if (pwm_out > 255)
      pwm_out = 255;
    digitalWrite(LEFT_WHEEL_DIR, HIGH); //
  }
  else
  {
    pwm_out = -1 * PWM + U0L;
    if (pwm_out > 255)
      pwm_out = 255;
    digitalWrite(LEFT_WHEEL_DIR, LOW);
  }
  analogWrite(LEFT_WHEEL_PWM, pwm_out);
}

void MoveRightMotor(int PWM)
{
  int pwm_out;
  if (PWM > 0)
  {
    pwm_out = PWM + U0R;
    if (pwm_out > 255)
      pwm_out = 255;
    digitalWrite(RIGHT_WHEEL_DIR, LOW);
  }
  else
  {
    pwm_out = -1 * PWM + U0R;
    if (pwm_out > 255)
      pwm_out = 255;
    digitalWrite(RIGHT_WHEEL_DIR, HIGH);
  }
  analogWrite(RIGHT_WHEEL_PWM, pwm_out);
}

//speed counter for left
void Code1()
{
#if MOTOR == DUAL_MOTOR
  int wheelDir = digitalRead(LEFT_WHEEL_B);
  if (wheelDir == LOW)
    count1++;
  else
    count1--;
#else
  count1++;
#endif
}

//speed counter for right
void Code2()
{
#if MOTOR == DUAL_MOTOR
  int wheelDir = digitalRead(RIGHT_WHEEL_B);
  if (wheelDir == HIGH) //HIGH)
    count2++;
  else
    count2--;
#else
  count2++;
#endif
}

long readLeftEncoder()
{
  return count1;
}

long readRightEncoder()
{

  return count2;
}

long getWheelsPosition()
{
  return count1 + count2;
}
