#include <maskor_ev3/touch_sensor.h>

namespace maskor_ev3 {

  touch_sensor::touch_sensor(address_type address) :
    sensor(address, { ev3_touch, nxt_touch })
  {
  }

} //end namespace
