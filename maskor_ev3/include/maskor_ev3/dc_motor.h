#ifndef _DC_MOTOR_H_
#define _DC_MOTOR_H_

// The DC motor class provides a uniform interface for using regular DC motors
// with no fancy controls or feedback. This includes LEGO MINDSTORMS RCX motors
// and LEGO Power Functions motors.

class dc_motor : protected device
{
 public:
  dc_motor(address_type address = OUTPUT_AUTO);

  using device::connected;
  using device::device_index;

  // Run the motor until another command is sent.
  static const std::string command_run_forever;

  // Run the motor for the amount of time specified in `time_sp`
  // and then stop the motor using the command specified by `stop_command`.
  static const std::string command_run_timed;

  // Run the motor at the duty cycle specified by `duty_cycle_sp`.
  // Unlike other run commands, changing `duty_cycle_sp` while running *will*
  // take effect immediately.
  static const std::string command_run_direct;

  // Stop any of the run commands before they are complete using the
  // command specified by `stop_command`.
  static const std::string command_stop;

  // With `normal` polarity, a positive duty cycle will
  // cause the motor to rotate clockwise.
  static const std::string polarity_normal;

  // With `inversed` polarity, a positive duty cycle will
  // cause the motor to rotate counter-clockwise.
  static const std::string polarity_inversed;

  // Power will be removed from the motor and it will freely coast to a stop.
  static const std::string stop_command_coast;

  // Power will be removed from the motor and a passive electrical load will
  // be placed on the motor. This is usually done by shorting the motor terminals
  // together. This load will absorb the energy from the rotation of the motors and
  // cause the motor to stop more quickly than coasting.
  static const std::string stop_command_brake;

  // Command: write-only
  // Sets the command for the motor. Possible values are `run-forever`, `run-timed` and
  // `stop`. Not all commands may be supported, so be sure to check the contents
  // of the `commands` attribute.
  auto set_command(std::string v) -> decltype(*this) {
    set_attr_string("command", v);
    return *this;
  }

  // Commands: read-only
  // Returns a list of commands supported by the motor
  // controller.
  mode_set commands() const { return get_attr_set("commands"); }

  // Driver Name: read-only
  // Returns the name of the motor driver that loaded this device. See the list
  // of [supported devices] for a list of drivers.
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Duty Cycle: read-only
  // Shows the current duty cycle of the PWM signal sent to the motor. Values
  // are -100 to 100 (-100% to 100%).
  int duty_cycle() const { return get_attr_int("duty_cycle"); }

  // Duty Cycle SP: read/write
  // Writing sets the duty cycle setpoint of the PWM signal sent to the motor.
  // Valid values are -100 to 100 (-100% to 100%). Reading returns the current
  // setpoint.
  int duty_cycle_sp() const { return get_attr_int("duty_cycle_sp"); }
  auto set_duty_cycle_sp(int v) -> decltype(*this) {
    set_attr_int("duty_cycle_sp", v);
    return *this;
  }

  // Polarity: read/write
  // Sets the polarity of the motor. Valid values are `normal` and `inversed`.
  std::string polarity() const { return get_attr_string("polarity"); }
  auto set_polarity(std::string v) -> decltype(*this) {
    set_attr_string("polarity", v);
    return *this;
  }

  // Address: read-only
  // Returns the name of the port that this motor is connected to.
  std::string address() const { return get_attr_string("address"); }

  // Ramp Down SP: read/write
  // Sets the time in milliseconds that it take the motor to ramp down from 100%
  // to 0%. Valid values are 0 to 10000 (10 seconds). Default is 0.
  int ramp_down_sp() const { return get_attr_int("ramp_down_sp"); }
  auto set_ramp_down_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_down_sp", v);
    return *this;
  }

  // Ramp Up SP: read/write
  // Sets the time in milliseconds that it take the motor to up ramp from 0% to
  // 100%. Valid values are 0 to 10000 (10 seconds). Default is 0.
  int ramp_up_sp() const { return get_attr_int("ramp_up_sp"); }
  auto set_ramp_up_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_up_sp", v);
    return *this;
  }

  // State: read-only
  // Gets a list of flags indicating the motor status. Possible
  // flags are `running` and `ramping`. `running` indicates that the motor is
  // powered. `ramping` indicates that the motor has not yet reached the
  // `duty_cycle_sp`.
  mode_set state() const { return get_attr_set("state"); }

  // Stop Command: write-only
  // Sets the stop command that will be used when the motor stops. Read
  // `stop_commands` to get the list of valid values.
  auto set_stop_command(std::string v) -> decltype(*this) {
    set_attr_string("stop_command", v);
    return *this;
  }

  // Stop Commands: read-only
  // Gets a list of stop commands. Valid values are `coast`
  // and `brake`.
  mode_set stop_commands() const { return get_attr_set("stop_commands"); }

  // Time SP: read/write
  // Writing specifies the amount of time the motor will run when using the
  // `run-timed` command. Reading returns the current value. Units are in
  // milliseconds.
  int time_sp() const { return get_attr_int("time_sp"); }
  auto set_time_sp(int v) -> decltype(*this) {
    set_attr_int("time_sp", v);
    return *this;
  }

  // Run the motor until another command is sent.
  void run_forever() { set_command("run-forever"); }

  // Run the motor for the amount of time specified in `time_sp`
  // and then stop the motor using the command specified by `stop_command`.
  void run_timed() { set_command("run-timed"); }

  // Run the motor at the duty cycle specified by `duty_cycle_sp`.
  // Unlike other run commands, changing `duty_cycle_sp` while running *will*
  // take effect immediately.
  void run_direct() { set_command("run-direct"); }

  // Stop any of the run commands before they are complete using the
  // command specified by `stop_command`.
  void stop() { set_command("stop"); }

 protected:
  std::string _port_name;
};


#endif //DC_MOTOR_H
