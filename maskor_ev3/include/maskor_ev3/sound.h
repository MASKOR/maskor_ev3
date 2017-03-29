#ifndef _SOUND_H_
#define _SOUND_H_


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


#endif //SOUND_H
