#ifndef _DEVICE_H_
#define _DEVICE_H_

/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 * Copyright (c) 2014 - Franz Detro
*/

// Generic device class.
// The sensor class provides a uniform interface for using most of the
// sensors available for the EV3. The various underlying device drivers will
// create a `lego-sensor` device for interacting with the sensors.
// 
// Sensors are primarily controlled by setting the `mode` and monitored by
// reading the `value<N>` attributes. Values can be converted to floating point
// if needed by `value<N>` / 10.0 ^ `decimals`.
// 
// Since the name of the `sensor<N>` device node does not correspond to the port
// that a sensor is plugged in to, you must look at the `address` attribute if
// you need to know which port a sensor is plugged in to. However, if you don't
// have more than one sensor of each type, you can just look for a matching
// `driver_name`. Then it will not matter which port a sensor is plugged in to
// your program will still work.

#include <maskor_ev3/common.h>
#include <string>
#include <string.h>
#include <map>
#include <dirent.h>


namespace maskor_ev3 {

class device
{
public:
  bool connect(const std::string &dir,
               const std::string &pattern,
               const std::map<std::string, std::set<std::string>> &match) noexcept;

  inline bool connected() const { return !_path.empty(); }

  int         device_index() const;

  int         get_attr_int   (const std::string &name) const;
  void        set_attr_int   (const std::string &name,
                              int value);
  std::string get_attr_string(const std::string &name) const;
  void        set_attr_string(const std::string &name,
                              const std::string &value);

  std::string get_attr_line  (const std::string &name) const;
  mode_set    get_attr_set   (const std::string &name,
                              std::string *pCur = nullptr) const;

  std::string get_attr_from_set(const std::string &name) const;

protected:
  std::string _path;
  mutable int _device_index = -1;
};

} //namespace maskor_ev3

#endif //DEVICE_H
