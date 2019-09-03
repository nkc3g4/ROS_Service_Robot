/************************************************
Description: 在arduino板上接蜂鸣器来生成各种声音反馈当前
  arduinoMega2560的运行状态。
Author: www.corvin.cn
History: 20180209: init this file;
*************************************************/

void initSoundPin()
{
  pinMode(SOUND_PIN, OUTPUT);
}

void basePowerOnBeep()
{
  for (int i = 200; i <= 600; i++)
  {
    tone(SOUND_PIN, i); 
    delay(3);
  }
  noTone(SOUND_PIN);
}

void basePowerOffBeep()
{
  for (int i = 600; i >= 200; i--)
  {
    tone(SOUND_PIN, i); 
    delay(3);
  }
  noTone(SOUND_PIN);
}

