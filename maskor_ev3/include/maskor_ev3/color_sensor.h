#ifndef _COLOR_SENSOR_H_
#define _COLOR_SENSOR_H_

#include <maskor_ev3/sensor.h>

// LEGO EV3 color sensor.

namespace maskor_ev3 {

class color_sensor : public sensor {
public:
  color_sensor(address_type address = INPUT_AUTO);

  // Reflected light. Red LED on.
  static const std::string mode_col_reflect;

  // Ambient light. Red LEDs off.
  static const std::string mode_col_ambient;

  // Color. All LEDs rapidly cycling, appears white.
  static const std::string mode_col_color;

  // Raw reflected. Red LED on
  static const std::string mode_ref_raw;

  // Raw Color Components. All LEDs rapidly cycling, appears white.
  static const std::string mode_rgb_raw;


  // Reflected light intensity as a percentage. Light on sensor is red.
  int reflected_light_intensity() {
    set_mode(mode_col_reflect);
    return value(0);
  }

  // Ambient light intensity. Light on sensor is dimly lit blue.
  int ambient_light_intensity() {
    set_mode(mode_col_ambient);
    return value(0);
  }

  // Color detected by the sensor, categorized by overall value.
  //   - 0: No color
  //   - 1: Black
  //   - 2: Blue
  //   - 3: Green
  //   - 4: Yellow
  //   - 5: Red
  //   - 6: White
  //   - 7: Brown
  int color() {
    set_mode(mode_col_color);
    return value(0);
  }

  // Red component of the detected color, in the range 0-1020.
  int red() {
    set_mode(mode_rgb_raw);
    return value(0);
  }

  // Green component of the detected color, in the range 0-1020.
  int green() {
    set_mode(mode_rgb_raw);
    return value(1);
  }

  // Blue component of the detected color, in the range 0-1020.
  int blue() {
    set_mode(mode_rgb_raw);
    return value(2);
  }

};

} // end namespace

#endif // COLOR_SENSOR_H
