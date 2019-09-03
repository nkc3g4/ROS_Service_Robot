/**************************************************************
Description:  Encoder definitions需要连接到arduinoMega2560的正确引脚上：
   Encoder A: connect interrupt 0, 1--[pin 2, 3];
   Encoder B: connect interrupt 2, 3--[pin 21, 20];
   Encoder C: connect intterupt 4, 5--[pin 19, 18]
Author: www.corvin.cn
History: 20180209:init this file;
**************************************************************/

static volatile long A_enc_pos = 0L;
static volatile long B_enc_pos = 0L;
static volatile long C_enc_pos = 0L;

/*init encoder connect pin*/
void initEncoders()
{
  pinMode(ENC_A_PIN_A, INPUT);
  pinMode(ENC_A_PIN_B, INPUT);
  attachInterrupt(3, encoderA_ISR, CHANGE);
  attachInterrupt(2, encoderA_ISR, CHANGE);

  pinMode(ENC_B_PIN_A, INPUT);
  pinMode(ENC_B_PIN_B, INPUT);
  attachInterrupt(0, encoderB_ISR, CHANGE);
  attachInterrupt(1, encoderB_ISR, CHANGE);

  pinMode(ENC_C_PIN_A, INPUT);
  pinMode(ENC_C_PIN_B, INPUT);
  attachInterrupt(5, encoderC_ISR, CHANGE);
  attachInterrupt(4, encoderC_ISR, CHANGE);
}

/* Interrupt routine for A encoder, taking care of actual counting */
void encoderA_ISR ()
{
  if (directionWheel(A_WHEEL) == BACKWARDS)
  {
    A_enc_pos--;
  }
  else 
  {
    A_enc_pos++;
  }
}

/* Interrupt routine for B encoder, taking care of actual counting */
void encoderB_ISR () 
{
  if (directionWheel(B_WHEEL) == BACKWARDS)
  {
    B_enc_pos--;
  }
  else 
  {
    B_enc_pos++;
  }
}

/* Interrupt routine for C encoder, taking care of actual counting */
void encoderC_ISR () 
{
  if (directionWheel(C_WHEEL) == BACKWARDS)
  {
    C_enc_pos--;
  }
  else 
  {
    C_enc_pos++;
  }
}

/* Wrap the encoder reading function */
long readEncoder(int i) 
{
  if (i == A_WHEEL)
  {
    return A_enc_pos;
  }
  else if (i == B_WHEEL)
  {
    return B_enc_pos;
  }
  else
  {
    return C_enc_pos;
  }
}

/* Wrap the encoder reset function */
void resetEncoders() {
  A_enc_pos = 0L;
  B_enc_pos = 0L;
  C_enc_pos = 0L;
}


