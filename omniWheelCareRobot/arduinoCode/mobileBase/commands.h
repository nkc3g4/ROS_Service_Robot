/***********************************************************************************
Description:Define single-letter commands that will be sent by the PC over the
   serial link.
Author: www.corvin.cn
History: 20180209:init this file,add CODE_VERSION command;
************************************************************************************/

#ifndef _COMMANDS_H_
#define _COMMANDS_H_

#define ANALOG_READ    'a'
#define GET_BAUDRATE   'b'
#define PIN_MODE       'c'
#define DIGITAL_READ   'd'
#define READ_ENCODERS  'e'
#define MOTOR_SPEEDS   'm'
#define RESET_ENCODERS 'r'
#define UPDATE_PID     'u'
#define DIGITAL_WRITE  'w'
#define ANALOG_WRITE   'x'
#define READ_PIDIN     'i'
#define READ_PIDOUT    'o'
#define SOUND_BEEP     'f'
#define CODE_VERSION   'v'

#define FORWARDS        true
#define BACKWARDS       false

#endif


