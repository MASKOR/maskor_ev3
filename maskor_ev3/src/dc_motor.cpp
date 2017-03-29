#include <maskor_ev3/dc_motor.h>

namespace maskor_ev3 {

dc_motor::dc_motor(address_type address)
{
  static const std::string _strClassDir { SYS_ROOT "/dc-motor/" };
  static const std::string _strPattern  { "motor" };

  connect(_strClassDir, _strPattern, {{ "address", { address }}});
}

}//end namespace
