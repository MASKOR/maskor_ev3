#include <maskor_ev3/i2c_sensor.h>

namespace maskor_ev3 {

i2c_sensor::i2c_sensor(address_type address) :
  sensor(address, { nxt_i2c_sensor })
{

}

} //end namespace
