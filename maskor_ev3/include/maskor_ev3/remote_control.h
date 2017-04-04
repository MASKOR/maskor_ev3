#ifndef _REMOTE_CONTROL_H_
#define _REMOTE_CONTROL_H_

/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 * Copyright (c) 2014 - Franz Detro
*/


#include <maskor_ev3/infrared_sensor.h>

namespace maskor_ev3 {

// EV3 remote control
class remote_control
{
public:
  remote_control(unsigned channel = 1);
  remote_control(infrared_sensor&, unsigned channel = 1);
  virtual ~remote_control();

  inline bool   connected() const { return _sensor->connected(); }
  inline unsigned channel() const { return _channel+1; }

  bool process();

  std::function<void (bool)> on_red_up;
  std::function<void (bool)> on_red_down;
  std::function<void (bool)> on_blue_up;
  std::function<void (bool)> on_blue_down;
  std::function<void (bool)> on_beacon;
  std::function<void (int)>  on_state_change;

  enum buttons
  {
    red_up    = (1 << 0),
    red_down  = (1 << 1),
    blue_up   = (1 << 2),
    blue_down = (1 << 3),
    beacon    = (1 << 4),
  };

protected:
  virtual void on_value_changed(int value);

  infrared_sensor *_sensor = nullptr;
  bool             _owns_sensor = false;
  unsigned         _channel = 0;
  int              _value = 0;
  int              _state = 0;
};

}//end namespace

#endif //REMOTE_CONTROL_H_
