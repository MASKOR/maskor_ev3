#ifndef _LIGHT_SENSOR_H_
#define _LIGHT_SENSOR_H_

/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 * Copyright (c) 2014 - Franz Detro
*/


#include <maskor_ev3/sensor.h>

namespace maskor_ev3 {

// LEGO NXT Light Sensor
class light_sensor : public sensor
{
public:
  light_sensor(address_type address = INPUT_AUTO);

  // Reflected light. LED on
  static const std::string mode_reflect;

  // Ambient light. LED off
  static const std::string mode_ambient;


  // A measurement of the reflected light intensity, as a percentage.
  float reflected_light_intensity() {
    set_mode(mode_reflect);
    return float_value(0);
  }

  // A measurement of the ambient light intensity, as a percentage.
  float ambient_light_intensity() {
    set_mode(mode_ambient);
    return float_value(0);
  }

};

}//end namespace

#endif //LIGHT_SENSOR_H
