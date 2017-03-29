#include <maskor_ev3/gyro_sensor.h>

namespace maskor_ev3 {

  gyro_sensor::gyro_sensor(address_type address) :
    sensor(address, { ev3_gyro })
  {
  }

}//end namespace
