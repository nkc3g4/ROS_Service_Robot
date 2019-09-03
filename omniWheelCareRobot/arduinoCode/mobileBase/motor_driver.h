/***************************************************************
Description: Motor driver function definitions - by James Nugen
Author: www.corvin.cn
History: 20180209: init this file;
****************************************************************/
#ifndef __MOTOR_DRIVER_H__
#define __MOTOR_DRIVER_H__

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int ASpeed, int BSpeed, int CSpeed);

#endif

