#ifndef _INFRARED_SENSOR_H_
#define _INFRARED_SENSOR_H_

#include <maskor_ev3/sensor.h>

namespace maskor_ev3 {

// LEGO EV3 infrared sensor.
class infrared_sensor : public sensor
{
public:
  infrared_sensor(address_type address = INPUT_AUTO);

  // Proximity
  static const std::string mode_ir_prox;

  // IR Seeker
  static const std::string mode_ir_seek;

  // IR Remote Control
  static const std::string mode_ir_remote;

  // IR Remote Control. State of the buttons is coded in binary
  static const std::string mode_ir_rem_a;

  // Calibration ???
  static const std::string mode_ir_cal;


  // A measurement of the distance between the sensor and the remote,
  // as a percentage. 100% is approximately 70cm/27in.
  int proximity() {
    set_mode(mode_ir_prox);
    return value(0);
  }

};

}//end namespace

#endif //INFRARED_SENSOR_H
