#ifndef _SOUND_SENSOR_H_
#define _SOUND_SENSOR_H_

// LEGO NXT Sound Sensor
class sound_sensor : public sensor
{
public:
  sound_sensor(address_type address = INPUT_AUTO);

  // Sound pressure level. Flat weighting
  static const std::string mode_db;

  // Sound pressure level. A weighting
  static const std::string mode_dba;


  // A measurement of the measured sound pressure level, as a
  // percent. Uses a flat weighting.
  float sound_pressure() {
    set_mode(mode_db);
    return float_value(0);
  }

  // A measurement of the measured sound pressure level, as a
  // percent. Uses A-weighting, which focuses on levels up to 55 dB.
  float sound_pressure_low() {
    set_mode(mode_dba);
    return float_value(0);
  }

};


#endif //SOUNDSENSOR_H
