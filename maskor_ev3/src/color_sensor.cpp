#include <maskor_ev3/color_sensor.h>

namespace maskor_ev3 {

color_sensor::color_sensor(address_type address) :
  sensor(address, { ev3_color })
{
}

}//end namespace
