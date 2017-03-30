#include <maskor_ev3/light_sensor.h>

namespace maskor_ev3 {

light_sensor::light_sensor(address_type address) :
  sensor(address, { nxt_light })
{
}

}//end namespace
