#include "device.h"

#include <string>
#include <vector>

class led : protected device
{
 public:
  led(std::string name);
  using device::connected;

  static led red_left;
  static led red_right;
  static led green_left;
  static led green_right;

  static std::vector<led*> left;
  static std::vector<led*> right;

  static std::vector<float> red;
  static std::vector<float> green;
  static std::vector<float> amber;
  static std::vector<float> orange;
  static std::vector<float> yellow;

  int max_brightness() const {return get_attr_int("max_brightness"); }
  int brightness() const { return get_attr_int("brightness"); }

  auto set_brightness(int v) -> decltype(*this)
  {
    set_attr_int("brightness", v);
    return *this;    
  }

  void on() { set_brightness(max_brightness()); }
  void off() {set_brightness(0); }

 protected:
  int _max_brightness = 0;
};
