#ifndef _PORT_H_
#define _PORT_H_

#include <maskor_ev3/device.h>

// The `lego-port` class provides an interface for working with input and
// output ports that are compatible with LEGO MINDSTORMS RCX/NXT/EV3, LEGO
// WeDo and LEGO Power Functions sensors and motors. Supported devices include
// the LEGO MINDSTORMS EV3 Intelligent Brick, the LEGO WeDo USB hub and
// various sensor multiplexers from 3rd party manufacturers.
// 
// Some types of ports may have multiple modes of operation. For example, the
// input ports on the EV3 brick can communicate with sensors using UART, I2C
// or analog validate signals - but not all at the same time. Therefore there
// are multiple modes available to connect to the different types of sensors.
// 
// In most cases, ports are able to automatically detect what type of sensor
// or motor is connected. In some cases though, this must be manually specified
// using the `mode` and `set_device` attributes. The `mode` attribute affects
// how the port communicates with the connected device. For example the input
// ports on the EV3 brick can communicate using UART, I2C or analog voltages,
// but not all at the same time, so the mode must be set to the one that is
// appropriate for the connected sensor. The `set_device` attribute is used to
// specify the exact type of sensor that is connected. Note: the mode must be
// correctly set before setting the sensor type.
// 
// Ports can be found at `/sys/class/lego-port/port<N>` where `<N>` is
// incremented each time a new port is registered. Note: The number is not
// related to the actual port at all - use the `address` attribute to find
// a specific port.

namespace maskor_ev3 {

class lego_port : protected device
{
 public:
  lego_port(address_type);

  using device::connected;
  using device::device_index;

  // Driver Name: read-only
  // Returns the name of the driver that loaded this device. You can find the
  // complete list of drivers in the [list of port drivers].
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Modes: read-only
  // Returns a list of the available modes of the port.
  mode_set modes() const { return get_attr_set("modes"); }

  // Mode: read/write
  // Reading returns the currently selected mode. Writing sets the mode.
  // Generally speaking when the mode changes any sensor or motor devices
  // associated with the port will be removed new ones loaded, however this
  // this will depend on the individual driver implementing this class.
  std::string mode() const { return get_attr_string("mode"); }
  auto set_mode(std::string v) -> decltype(*this) {
    set_attr_string("mode", v);
    return *this;
  }

  // Address: read-only
  // Returns the name of the port. See individual driver documentation for
  // the name that will be returned.
  std::string address() const { return get_attr_string("address"); }

  // Set Device: write-only
  // For modes that support it, writing the name of a driver will cause a new
  // device to be registered for that driver and attached to this port. For
  // example, since NXT/Analog sensors cannot be auto-detected, you must use
  // this attribute to load the correct driver. Returns -EOPNOTSUPP if setting a
  // device is not supported.
  auto set_set_device(std::string v) -> decltype(*this) {
    set_attr_string("set_device", v);
    return *this;
  }

  // Status: read-only
  // In most cases, reading status will return the same value as `mode`. In
  // cases where there is an `auto` mode additional values may be returned,
  // such as `no-device` or `error`. See individual port driver documentation
  // for the full list of possible values.
  std::string status() const { return get_attr_string("status"); }

 protected:
  lego_port() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
};

}//end namespace

#endif //PORT_H
