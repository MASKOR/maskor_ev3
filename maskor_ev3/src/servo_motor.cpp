#include <maskor_ev3/servo_motor.h>

namespace maskor_ev3 {

servo_motor::servo_motor(address_type address)
{
  static const std::string _strClassDir { SYS_ROOT "/servo-motor/" };
  static const std::string _strPattern  { "motor" };

  connect(_strClassDir, _strPattern, {{ "address", { address }}});
}

} //end namespace
