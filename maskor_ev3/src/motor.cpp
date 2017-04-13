#include <maskor_ev3/motor.h>

namespace maskor_ev3 {

  motor::motor(address_type address)
  {
    connect({{ "address", { address } }});
  }

  //-----------------------------------------------------------------------------

  motor::motor(address_type address, const motor_type &t)
  {
    connect({{ "address", { address } }, { "driver_name", { t }}});
  }

  //-----------------------------------------------------------------------------

  void motor::move_forward()
  {
    
  }

  void motor::move_left()
  {

  }

  void motor::move_right()
  {

  }

  void motor::move_backward()
  {

  }
  
  bool motor::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
  {
    static const std::string _strClassDir { SYS_ROOT "/tacho-motor/" };
    static const std::string _strPattern  { "motor" };

    try
      {
	return device::connect(_strClassDir, _strPattern, match);
      }
    catch (...) { }

    _path.clear();

    return false;
  }

}//end namespace
