#include <maskor_ev3/power_supply.h>

namespace maskor_ev3 {

power_supply::power_supply(std::string name)
{
  static const std::string _strClassDir { SYS_ROOT "/power_supply/" };

  if (name.empty())
    name = "legoev3-battery";

  connect(_strClassDir, name, std::map<std::string, std::set<std::string>>());
}

}//end namespace
