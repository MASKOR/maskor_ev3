#ifndef _LED_H_
#define _LED_H_

#include <maskor_ev3/device.h>
#include <thread>

// Any device controlled by the generic LED driver.
// See https://www.kernel.org/doc/Documentation/leds/leds-class.txt
// for more details.

namespace maskor_ev3 {

class led : protected device
{
 public:
  led(std::string name);
  using device::connected;

  // Max Brightness: read-only
  // Returns the maximum allowable brightness value.
  int max_brightness() const { return get_attr_int("max_brightness"); }

  // Brightness: read/write
  // Sets the brightness level. Possible values are from 0 to `max_brightness`.
  int brightness() const { return get_attr_int("brightness"); }
  auto set_brightness(int v) -> decltype(*this) {
    set_attr_int("brightness", v);
    return *this;
  }

  // Triggers: read-only
  // Returns a list of available triggers.
  mode_set triggers() const { return get_attr_set("trigger"); }

  // Trigger: read/write
  // Sets the led trigger. A trigger
  // is a kernel based source of led events. Triggers can either be simple or
  // complex. A simple trigger isn't configurable and is designed to slot into
  // existing subsystems with minimal additional code. Examples are the `ide-disk` and
  // `nand-disk` triggers.
  // 
  // Complex triggers whilst available to all LEDs have LED specific
  // parameters and work on a per LED basis. The `timer` trigger is an example.
  // The `timer` trigger will periodically change the LED brightness between
  // 0 and the current brightness setting. The `on` and `off` time can
  // be specified via `delay_{on,off}` attributes in milliseconds.
  // You can change the brightness value of a LED independently of the timer
  // trigger. However, if you set the brightness value to 0 it will
  // also disable the `timer` trigger.
  std::string trigger() const { return get_attr_from_set("trigger"); }
  auto set_trigger(std::string v) -> decltype(*this) {
    set_attr_string("trigger", v);
    return *this;
  }

  // Delay On: read/write
  // The `timer` trigger will periodically change the LED brightness between
  // 0 and the current brightness setting. The `on` time can
  // be specified via `delay_on` attribute in milliseconds.
  int delay_on() const { return get_attr_int("delay_on"); }
  auto set_delay_on(int v) -> decltype(*this) {
    set_attr_int("delay_on", v);
    return *this;
  }

  // Delay Off: read/write
  // The `timer` trigger will periodically change the LED brightness between
  // 0 and the current brightness setting. The `off` time can
  // be specified via `delay_off` attribute in milliseconds.
  int delay_off() const { return get_attr_int("delay_off"); }
  auto set_delay_off(int v) -> decltype(*this) {
    set_attr_int("delay_off", v);
    return *this;
  }

  // Gets the LED's brightness as a percentage (0-1) of the maximum.
  float brightness_pct() const {
    return static_cast<float>(brightness()) / max_brightness();
  }

  // Sets the LED's brightness as a percentage (0-1) of the maximum.
  auto set_brightness_pct(float v) -> decltype(*this) {
    return set_brightness(v * max_brightness());
  }

  // Turns the led on by setting its brightness to the maximum level.
  void on()  { set_brightness(max_brightness()); }

  // Turns the led off.
  void off() { set_brightness(0); }

  // Enables timer trigger and sets delay_on and delay_off attributes to the
  // provided values (in milliseconds).
  void flash(unsigned on_ms, unsigned off_ms);

#ifdef EV3DEV_PLATFORM_BRICKPI
  static led blue_led1;
  static led blue_led2;

  static std::vector<led*> led1;
  static std::vector<led*> led2;

  static std::vector<float> blue;

#else
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

#endif

  // Assigns to each led in `group` corresponding brightness percentage from `color`.
  static void set_color(const std::vector<led*> &group, const std::vector<float> &color);

  static void all_off();

 protected:
  int _max_brightness = 0;
};

}//end namespace

#endif //LED_H
