#include <maskor_ev3/port.h>

namespace maskor_ev3 {

lego_port::lego_port(address_type address)
{
  connect({{ "address", { address } }});
}

//-----------------------------------------------------------------------------

bool lego_port::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
{
  static const std::string _strClassDir { SYS_ROOT "/lego-port/" };
  static const std::string _strPattern  { "port" };

  try
  {
    return device::connect(_strClassDir, _strPattern, match);
  }
  catch (...) { }

  _path.clear();

  return false;
}
}//end namespace
