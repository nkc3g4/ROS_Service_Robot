/**************************************************************
Description: Encoder driver function definitions - by James Nugen
Author: www.corvin.cn
History: 20180209:init this file;
*************************************************************/
#ifndef __ENCODER_DRIVER_H__
#define __ENCODER_DRIVER_H__

//below can be changed, but should be PORTD pins;
//A wheel encode pin
#define ENC_A_PIN_A  20 //pin 20 -- interrupt 3   
#define ENC_A_PIN_B  21 //pin 21 -- interrupt 2   

//B wheel encode pin
#define ENC_B_PIN_A  2  //pin 2 -- interrupt 0
#define ENC_B_PIN_B  3  //pin 3 -- interrupt 1

//C wheel encode pin
#define ENC_C_PIN_A  18  //pin 18 -- interrupt 5
#define ENC_C_PIN_B  19  //pin 19 -- interrupt 4 

#define A_WHEEL      1
#define B_WHEEL      2
#define C_WHEEL      3

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

void initEncoders();

#endif

