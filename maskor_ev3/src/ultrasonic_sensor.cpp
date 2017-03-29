#include <maskor_ev3/ultrasonic_sensor.h>

namespace maskor_ev3 {

  ultrasonic_sensor::ultrasonic_sensor(address_type address) :
    sensor(address, { ev3_ultrasonic, nxt_ultrasonic })
  {
  }

}//end namespace
