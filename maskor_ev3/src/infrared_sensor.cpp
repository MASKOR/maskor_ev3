#include <maskor_ev3/infrared_sensor.h>

namespace maskor_ev3 {

  infrared_sensor::infrared_sensor(address_type address) :
    sensor(address, { ev3_infrared })
  {
  }

}//end namespace
