#ifndef _SOUND_H_
#define _SOUND_H_

/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 * Copyright (c) 2014 - Franz Detro
*/

#include<string>
#include<vector>

namespace maskor_ev3 {

// EV3 Sound
class sound
{
public:
  static void beep(const std::string &args = "", bool bSynchronous = false);
  static void tone(float frequency, float ms, bool bSynchronous = false);
  static void tone(const std::vector< std::vector<float> > &sequence, bool bSynchronous = false);
  static void play(const std::string &soundfile, bool bSynchronous = false);
  static void speak(const std::string &text, bool bSynchronous = false);
};

}//end namespace

#endif //SOUND_H
