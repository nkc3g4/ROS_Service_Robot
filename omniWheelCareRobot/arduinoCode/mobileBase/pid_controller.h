/************************************************************************
Description:Functions and type-defs for PID control.
   Taken mostly from Mike Ferguson's ArbotiX code which lives at:
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
Author: www.corvin.cn
History: 20180209: init this file;
**************************************************************************/
#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_

/* PID setpoint info For a Motor */
typedef struct {
  double TargetTicksPerFrame;    // target speed in ticks per frame
  long Encoder;                  // encoder count
  long PrevEnc;                  // last encoder count

  /*
    Using previous input (PrevInput) instead of PrevError to avoid derivative kick,
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-derivative-kick/
  */
  int PrevInput;                // last input

  /*
    Using integrated term (ITerm) instead of integrated error (Ierror),
    to allow tuning changes,
    see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-tuning-changes/
  */
  int ITerm;                    //integrated term
  long output;                  //last motor setting
}SetPointInfo;


void resetPID();
void updatePID();
long readPidIn(int wheel);
long readPidOut(int wheel);
void setMoveStatus(char state);
void doAWheelPID(SetPointInfo *p);
void doBWheelPID(SetPointInfo *p);
void doCWheelPID(SetPointInfo *p);
void setWheelPIDTarget(int value_A, int value_B, int value_C);
void updatePIDParam(int index, int kp, int kd, int ki, int ko);

#define PID_A     1
#define PID_B     2
#define PID_C     3

#endif

