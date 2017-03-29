#ifndef _TOUCH_SENSOR_H_
#define _TOUCH_SENSOR_H_


// Touch Sensor
class touch_sensor : public sensor
{
public:
  touch_sensor(address_type address = INPUT_AUTO);

  // Button state
  static const std::string mode_touch;


  // A boolean indicating whether the current touch sensor is being
  // pressed.
  bool is_pressed() {
    set_mode(mode_touch);
    return value(0);
  }

};


#endif
