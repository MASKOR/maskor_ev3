#ifndef _SERVO_MOTOR_H_
#define _SERVO_MOTOR_H

#include <maskor_ev3/device.h>

// The servo motor class provides a uniform interface for using hobby type
// servo motors.

namespace maskor_ev3 {

class servo_motor : protected device
{
 public:
  servo_motor(address_type address = OUTPUT_AUTO);

  using device::connected;
  using device::device_index;

  // Drive servo to the position set in the `position_sp` attribute.
  static const std::string command_run;

  // Remove power from the motor.
  static const std::string command_float;

  // With `normal` polarity, a positive duty cycle will
  // cause the motor to rotate clockwise.
  static const std::string polarity_normal;

  // With `inversed` polarity, a positive duty cycle will
  // cause the motor to rotate counter-clockwise.
  static const std::string polarity_inversed;

  // Command: write-only
  // Sets the command for the servo. Valid values are `run` and `float`. Setting
  // to `run` will cause the servo to be driven to the position_sp set in the
  // `position_sp` attribute. Setting to `float` will remove power from the motor.
  auto set_command(std::string v) -> decltype(*this) {
    set_attr_string("command", v);
    return *this;
  }

  // Driver Name: read-only
  // Returns the name of the motor driver that loaded this device. See the list
  // of [supported devices] for a list of drivers.
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Max Pulse SP: read/write
  // Used to set the pulse size in milliseconds for the signal that tells the
  // servo to drive to the maximum (clockwise) position_sp. Default value is 2400.
  // Valid values are 2300 to 2700. You must write to the position_sp attribute for
  // changes to this attribute to take effect.
  int max_pulse_sp() const { return get_attr_int("max_pulse_sp"); }
  auto set_max_pulse_sp(int v) -> decltype(*this) {
    set_attr_int("max_pulse_sp", v);
    return *this;
  }

  // Mid Pulse SP: read/write
  // Used to set the pulse size in milliseconds for the signal that tells the
  // servo to drive to the mid position_sp. Default value is 1500. Valid
  // values are 1300 to 1700. For example, on a 180 degree servo, this would be
  // 90 degrees. On continuous rotation servo, this is the 'neutral' position_sp
  // where the motor does not turn. You must write to the position_sp attribute for
  // changes to this attribute to take effect.
  int mid_pulse_sp() const { return get_attr_int("mid_pulse_sp"); }
  auto set_mid_pulse_sp(int v) -> decltype(*this) {
    set_attr_int("mid_pulse_sp", v);
    return *this;
  }

  // Min Pulse SP: read/write
  // Used to set the pulse size in milliseconds for the signal that tells the
  // servo to drive to the miniumum (counter-clockwise) position_sp. Default value
  // is 600. Valid values are 300 to 700. You must write to the position_sp
  // attribute for changes to this attribute to take effect.
  int min_pulse_sp() const { return get_attr_int("min_pulse_sp"); }
  auto set_min_pulse_sp(int v) -> decltype(*this) {
    set_attr_int("min_pulse_sp", v);
    return *this;
  }

  // Polarity: read/write
  // Sets the polarity of the servo. Valid values are `normal` and `inversed`.
  // Setting the value to `inversed` will cause the position_sp value to be
  // inversed. i.e `-100` will correspond to `max_pulse_sp`, and `100` will
  // correspond to `min_pulse_sp`.
  std::string polarity() const { return get_attr_string("polarity"); }
  auto set_polarity(std::string v) -> decltype(*this) {
    set_attr_string("polarity", v);
    return *this;
  }

  // Address: read-only
  // Returns the name of the port that this motor is connected to.
  std::string address() const { return get_attr_string("address"); }

  // Position SP: read/write
  // Reading returns the current position_sp of the servo. Writing instructs the
  // servo to move to the specified position_sp. Units are percent. Valid values
  // are -100 to 100 (-100% to 100%) where `-100` corresponds to `min_pulse_sp`,
  // `0` corresponds to `mid_pulse_sp` and `100` corresponds to `max_pulse_sp`.
  int position_sp() const { return get_attr_int("position_sp"); }
  auto set_position_sp(int v) -> decltype(*this) {
    set_attr_int("position_sp", v);
    return *this;
  }

  // Rate SP: read/write
  // Sets the rate_sp at which the servo travels from 0 to 100.0% (half of the full
  // range of the servo). Units are in milliseconds. Example: Setting the rate_sp
  // to 1000 means that it will take a 180 degree servo 2 second to move from 0
  // to 180 degrees. Note: Some servo controllers may not support this in which
  // case reading and writing will fail with `-EOPNOTSUPP`. In continuous rotation
  // servos, this value will affect the rate_sp at which the speed ramps up or down.
  int rate_sp() const { return get_attr_int("rate_sp"); }
  auto set_rate_sp(int v) -> decltype(*this) {
    set_attr_int("rate_sp", v);
    return *this;
  }

  // State: read-only
  // Returns a list of flags indicating the state of the servo.
  // Possible values are:
  // * `running`: Indicates that the motor is powered.
  mode_set state() const { return get_attr_set("state"); }

  // Drive servo to the position set in the `position_sp` attribute.
  void run() { set_command("run"); }

  // Remove power from the motor.
  void float_() { set_command("float"); }
};

}//end namespace

#endif //SERVO_MOTOR_H
