/**********************************************************
Description: 下位机arduinoMega2560上的蜂鸣器模块头文件，
  定义蜂鸣器连接的引脚，各种声音的宏定义。
Author: www.corvin.cn
History: 20180209: init this file;
***********************************************************/

#ifndef _SOUND_H_
#define _SOUND_H_

#define  SOUND_PIN      53

#define  BASE_POWEROFF_BEEP  0
#define  BASE_POWERON_BEEP   1

void initSoundPin();
void basePowerOnBeep();
void basePowerOffBeep();

#endif
