/***************************************************************
Description: Motor driver definitions;
Author: www.corvin.cn
Hisotry: 20180209: init this file;
*************************************************************/
//three motors control pin define
static const int A_IN1  = 26;
static const int A_IN2  = 28;
static const int A_PWM  = 4;   //A wheel pwm pin

static const int B_IN1  = 30;
static const int B_IN2  = 32;
static const int B_PWM  = 6;   //B wheel pwm pin

static const int C_IN1  = 24;
static const int C_IN2  = 22;
static const int C_PWM  = 5;    //C wheel pwm pin

static boolean direcA = FORWARDS;
static boolean direcB = FORWARDS;
static boolean direcC = FORWARDS;

/*
 * get wheel run direction
*/
boolean directionWheel(int wheel) 
{
  if (wheel == A_WHEEL) 
  {
    return direcA;
  }
  else if (wheel == B_WHEEL)
  {
    return direcB;
  }
  else 
  {
    return direcC;
  }
}

/* Wrap the motor driver initialization,
   set all the motor control pins to outputs **/
void initMotorController()
{
  pinMode(A_IN1, OUTPUT);
  pinMode(A_IN2, OUTPUT);
  pinMode(A_PWM, OUTPUT);

  pinMode(B_IN1, OUTPUT);
  pinMode(B_IN2, OUTPUT);
  pinMode(B_PWM, OUTPUT);

  pinMode(C_IN1, OUTPUT);
  pinMode(C_IN2, OUTPUT);
  pinMode(C_PWM, OUTPUT);
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int wheel, int spd)
{
  if (spd > MAX_PWM) 
  {
    spd = MAX_PWM;
  }
  if (spd < -MAX_PWM) 
  {
    spd = -1 * MAX_PWM;
  }

  if (wheel == A_WHEEL)
  {
    if (spd >= 0) 
    {
      direcA = FORWARDS;
      digitalWrite(A_IN1, LOW);
      digitalWrite(A_IN2, HIGH);
      analogWrite(A_PWM, spd);
    }
    else if (spd < 0) 
    {
      direcA = BACKWARDS;
      digitalWrite(A_IN1, HIGH);
      digitalWrite(A_IN2, LOW);
      analogWrite(A_PWM, -spd);
    }
  }
  else if (wheel == B_WHEEL)
  {
    if (spd >= 0) 
    {
      direcB = FORWARDS;
      digitalWrite(B_IN1, LOW);
      digitalWrite(B_IN2, HIGH);
      analogWrite(B_PWM, spd);
    }
    else if (spd < 0) 
    {
      direcB = BACKWARDS;
      digitalWrite(B_IN1, HIGH);
      digitalWrite(B_IN2, LOW);
      analogWrite(B_PWM, -spd);
    }
  }
  else
  {
    if (spd >= 0) 
    {
      direcC = FORWARDS;
      digitalWrite(C_IN1, LOW);
      digitalWrite(C_IN2, HIGH);
      analogWrite(C_PWM, spd);
    }
    else if (spd < 0) 
    {
      direcC = BACKWARDS;
      digitalWrite(C_IN1, HIGH);
      digitalWrite(C_IN2, LOW);
      analogWrite(C_PWM, -spd);
    }
  }
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int ASpeed, int BSpeed, int CSpeed)
{
  setMotorSpeed(A_WHEEL, ASpeed);
  setMotorSpeed(B_WHEEL, BSpeed);
  setMotorSpeed(C_WHEEL, CSpeed);
}

