#ifndef _ULTRASONIC_SENSOR_H_
#define _ULTRASONIC_SENSOR_H_

/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 * Copyright (c) 2014 - Franz Detro
*/

#include <maskor_ev3/sensor.h>

namespace maskor_ev3 {

// LEGO EV3 ultrasonic sensor.
class ultrasonic_sensor : public sensor
{
public:
  ultrasonic_sensor(address_type address = INPUT_AUTO);

  // Continuous measurement in centimeters.
  static const std::string mode_us_dist_cm;

  // Continuous measurement in inches.
  static const std::string mode_us_dist_in;

  // Listen.
  static const std::string mode_us_listen;

  // Single measurement in centimeters.
  static const std::string mode_us_si_cm;

  // Single measurement in inches.
  static const std::string mode_us_si_in;


  // Measurement of the distance detected by the sensor,
  // in centimeters.
  float distance_centimeters() {
    set_mode(mode_us_dist_cm);
    return float_value(0);
  }

  // Measurement of the distance detected by the sensor,
  // in inches.
  float distance_inches() {
    set_mode(mode_us_dist_in);
    return float_value(0);
  }

  // Value indicating whether another ultrasonic sensor could
  // be heard nearby.
  bool other_sensor_present() {
    set_mode(mode_us_listen);
    return value(0);
  }

};

}//end namespace

#endif //ULTRASONIC_SENSOR_H
