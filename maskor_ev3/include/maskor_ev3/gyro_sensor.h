#ifndef _GYRO_SENSOR_H_
#define _GYRO_SENSOR_H_

/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 * Copyright (c) 2014 - Franz Detro
*/


#include <maskor_ev3/sensor.h>

namespace maskor_ev3 {

// LEGO EV3 gyro sensor.
class gyro_sensor : public sensor
{
public:
  gyro_sensor(address_type address = INPUT_AUTO);

  // Angle
  static const std::string mode_gyro_ang;

  // Rotational speed
  static const std::string mode_gyro_rate;

  // Raw sensor value
  static const std::string mode_gyro_fas;

  // Angle and rotational speed
  static const std::string mode_gyro_g_a;

  // Calibration ???
  static const std::string mode_gyro_cal;


  // The number of degrees that the sensor has been rotated
  // since it was put into this mode.
  int angle() {
    set_mode(mode_gyro_ang);
    return value(0);
  }

  // The rate at which the sensor is rotating, in degrees/second.
  int rate() {
    set_mode(mode_gyro_rate);
    return value(0);
  }

  int g_a() {
    set_mode(mode_gyro_g_a);
    return value(0), value(1);
  }
};

}//end namespace

#endif //GYRO_SENSOR_H
