#ifndef _GYRO_SENSOR_H_
#define _GYRO_SENSOR_H_

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

};

#endif //GYRO_SENSOR_H
