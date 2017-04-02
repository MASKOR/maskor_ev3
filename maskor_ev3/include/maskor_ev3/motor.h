#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <maskor_ev3/device.h>

// The motor class provides a uniform interface for using motors with
// positional and directional feedback such as the EV3 and NXT motors.
// This feedback allows for precise control of the motors. This is the
// most common type of motor, so we just call it `motor`.

namespace maskor_ev3 {

class motor : protected device {
public:
  typedef device_type motor_type;

  motor(address_type);
  motor(address_type, const motor_type&);

  static const motor_type motor_large;
  static const motor_type motor_medium;

  using device::connected;
  using device::device_index;

//~autogen generic-declare-property-value classes.motor>currentClass

  // Run the motor until another command is sent.
  static const std::string command_run_forever;

  // Run to an absolute position specified by `position_sp` and then
  // stop using the command specified in `stop_command`.
  static const std::string command_run_to_abs_pos;

  // Run to a position relative to the current `position` value.
  // The new position will be current `position` + `position_sp`.
  // When the new position is reached, the motor will stop using
  // the command specified by `stop_command`.
  static const std::string command_run_to_rel_pos;

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

  // Reset all of the motor parameter attributes to their default value.
  // This will also have the effect of stopping the motor.
  static const std::string command_reset;

  // Sets the normal polarity of the rotary encoder.
  static const std::string encoder_polarity_normal;

  // Sets the inversed polarity of the rotary encoder.
  static const std::string encoder_polarity_inversed;

  // With `normal` polarity, a positive duty cycle will
  // cause the motor to rotate clockwise.
  static const std::string polarity_normal;

  // With `inversed` polarity, a positive duty cycle will
  // cause the motor to rotate counter-clockwise.
  static const std::string polarity_inversed;

  // The motor controller will vary the power supplied to the motor
  // to try to maintain the speed specified in `speed_sp`.
  static const std::string speed_regulation_on;

  // The motor controller will use the power specified in `duty_cycle_sp`.
  static const std::string speed_regulation_off;

  // Power will be removed from the motor and it will freely coast to a stop.
  static const std::string stop_command_coast;

  // Power will be removed from the motor and a passive electrical load will
  // be placed on the motor. This is usually done by shorting the motor terminals
  // together. This load will absorb the energy from the rotation of the motors and
  // cause the motor to stop more quickly than coasting.
  static const std::string stop_command_brake;

  // Does not remove power from the motor. Instead it actively try to hold the motor
  // at the current position. If an external force tries to turn the motor, the motor
  // will ``push back`` to maintain its position.
  static const std::string stop_command_hold;


//~autogen

//~autogen generic-get-set classes.motor>currentClass

  // Command: write-only
  // Sends a command to the motor controller. See `commands` for a list of
  // possible values.
  auto set_command(std::string v) -> decltype(*this) {
    set_attr_string("command", v);
    return *this;
  }

  // Commands: read-only
  // Returns a list of commands that are supported by the motor
  // controller. Possible values are `run-forever`, `run-to-abs-pos`, `run-to-rel-pos`,
  // `run-timed`, `run-direct`, `stop` and `reset`. Not all commands may be supported.
  // 
  // - `run-forever` will cause the motor to run until another command is sent.
  // - `run-to-abs-pos` will run to an absolute position specified by `position_sp`
  //   and then stop using the command specified in `stop_command`.
  // - `run-to-rel-pos` will run to a position relative to the current `position` value.
  //   The new position will be current `position` + `position_sp`. When the new
  //   position is reached, the motor will stop using the command specified by `stop_command`.
  // - `run-timed` will run the motor for the amount of time specified in `time_sp`
  //   and then stop the motor using the command specified by `stop_command`.
  // - `run-direct` will run the motor at the duty cycle specified by `duty_cycle_sp`.
  //   Unlike other run commands, changing `duty_cycle_sp` while running *will*
  //   take effect immediately.
  // - `stop` will stop any of the run commands before they are complete using the
  //   command specified by `stop_command`.
  // - `reset` will reset all of the motor parameter attributes to their default value.
  //   This will also have the effect of stopping the motor.
  mode_set commands() const { return get_attr_set("commands"); }

  // Count Per Rot: read-only
  // Returns the number of tacho counts in one rotation of the motor. Tacho counts
  // are used by the position and speed attributes, so you can use this value
  // to convert rotations or degrees to tacho counts. In the case of linear
  // actuators, the units here will be counts per centimeter.
  int count_per_rot() const { return get_attr_int("count_per_rot"); }

  // Driver Name: read-only
  // Returns the name of the driver that provides this tacho motor device.
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Duty Cycle: read-only
  // Returns the current duty cycle of the motor. Units are percent. Values
  // are -100 to 100.
  int duty_cycle() const { return get_attr_int("duty_cycle"); }

  // Duty Cycle SP: read/write
  // Writing sets the duty cycle setpoint. Reading returns the current value.
  // Units are in percent. Valid values are -100 to 100. A negative value causes
  // the motor to rotate in reverse. This value is only used when `speed_regulation`
  // is off.
  int duty_cycle_sp() const { return get_attr_int("duty_cycle_sp"); }
  auto set_duty_cycle_sp(int v) -> decltype(*this) {
    set_attr_int("duty_cycle_sp", v);
    return *this;
  }

  // Encoder Polarity: read/write
  // Sets the polarity of the rotary encoder. This is an advanced feature to all
  // use of motors that send inversed encoder signals to the EV3. This should
  // be set correctly by the driver of a device. It You only need to change this
  // value if you are using a unsupported device. Valid values are `normal` and
  // `inversed`.
  std::string encoder_polarity() const { return get_attr_string("encoder_polarity"); }
  auto set_encoder_polarity(std::string v) -> decltype(*this) {
    set_attr_string("encoder_polarity", v);
    return *this;
  }

  // Polarity: read/write
  // Sets the polarity of the motor. With `normal` polarity, a positive duty
  // cycle will cause the motor to rotate clockwise. With `inversed` polarity,
  // a positive duty cycle will cause the motor to rotate counter-clockwise.
  // Valid values are `normal` and `inversed`.
  std::string polarity() const { return get_attr_string("polarity"); }
  auto set_polarity(std::string v) -> decltype(*this) {
    set_attr_string("polarity", v);
    return *this;
  }

  // Address: read-only
  // Returns the name of the port that this motor is connected to.
  std::string address() const { return get_attr_string("address"); }

  // Position: read/write
  // Returns the current position of the motor in pulses of the rotary
  // encoder. When the motor rotates clockwise, the position will increase.
  // Likewise, rotating counter-clockwise causes the position to decrease.
  // Writing will set the position to that value.
  int position() const { return get_attr_int("position"); }
  auto set_position(int v) -> decltype(*this) {
    set_attr_int("position", v);
    return *this;
  }

  // Position P: read/write
  // The proportional constant for the position PID.
  int position_p() const { return get_attr_int("hold_pid/Kp"); }
  auto set_position_p(int v) -> decltype(*this) {
    set_attr_int("hold_pid/Kp", v);
    return *this;
  }

  // Position I: read/write
  // The integral constant for the position PID.
  int position_i() const { return get_attr_int("hold_pid/Ki"); }
  auto set_position_i(int v) -> decltype(*this) {
    set_attr_int("hold_pid/Ki", v);
    return *this;
  }

  // Position D: read/write
  // The derivative constant for the position PID.
  int position_d() const { return get_attr_int("hold_pid/Kd"); }
  auto set_position_d(int v) -> decltype(*this) {
    set_attr_int("hold_pid/Kd", v);
    return *this;
  }

  // Position SP: read/write
  // Writing specifies the target position for the `run-to-abs-pos` and `run-to-rel-pos`
  // commands. Reading returns the current value. Units are in tacho counts. You
  // can use the value returned by `counts_per_rot` to convert tacho counts to/from
  // rotations or degrees.
  int position_sp() const { return get_attr_int("position_sp"); }
  auto set_position_sp(int v) -> decltype(*this) {
    set_attr_int("position_sp", v);
    return *this;
  }

  // Speed: read-only
  // Returns the current motor speed in tacho counts per second. Not, this is
  // not necessarily degrees (although it is for LEGO motors). Use the `count_per_rot`
  // attribute to convert this value to RPM or deg/sec.
  int speed() const { return get_attr_int("speed"); }

  // Speed SP: read/write
  // Writing sets the target speed in tacho counts per second used when `speed_regulation`
  // is on. Reading returns the current value.  Use the `count_per_rot` attribute
  // to convert RPM or deg/sec to tacho counts per second.
  int speed_sp() const { return get_attr_int("speed_sp"); }
  auto set_speed_sp(int v) -> decltype(*this) {
    set_attr_int("speed_sp", v);
    return *this;
  }

  // Ramp Up SP: read/write
  // Writing sets the ramp up setpoint. Reading returns the current value. Units
  // are in milliseconds. When set to a value > 0, the motor will ramp the power
  // sent to the motor from 0 to 100% duty cycle over the span of this setpoint
  // when starting the motor. If the maximum duty cycle is limited by `duty_cycle_sp`
  // or speed regulation, the actual ramp time duration will be less than the setpoint.
  int ramp_up_sp() const { return get_attr_int("ramp_up_sp"); }
  auto set_ramp_up_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_up_sp", v);
    return *this;
  }

  // Ramp Down SP: read/write
  // Writing sets the ramp down setpoint. Reading returns the current value. Units
  // are in milliseconds. When set to a value > 0, the motor will ramp the power
  // sent to the motor from 100% duty cycle down to 0 over the span of this setpoint
  // when stopping the motor. If the starting duty cycle is less than 100%, the
  // ramp time duration will be less than the full span of the setpoint.
  int ramp_down_sp() const { return get_attr_int("ramp_down_sp"); }
  auto set_ramp_down_sp(int v) -> decltype(*this) {
    set_attr_int("ramp_down_sp", v);
    return *this;
  }

  // Speed Regulation Enabled: read/write
  // Turns speed regulation on or off. If speed regulation is on, the motor
  // controller will vary the power supplied to the motor to try to maintain the
  // speed specified in `speed_sp`. If speed regulation is off, the controller
  // will use the power specified in `duty_cycle_sp`. Valid values are `on` and
  // `off`.
  std::string speed_regulation_enabled() const { return get_attr_string("speed_regulation"); }
  auto set_speed_regulation_enabled(std::string v) -> decltype(*this) {
    set_attr_string("speed_regulation", v);
    return *this;
  }

  // Speed Regulation P: read/write
  // The proportional constant for the speed regulation PID.
  int speed_regulation_p() const { return get_attr_int("speed_pid/Kp"); }
  auto set_speed_regulation_p(int v) -> decltype(*this) {
    set_attr_int("speed_pid/Kp", v);
    return *this;
  }

  // Speed Regulation I: read/write
  // The integral constant for the speed regulation PID.
  int speed_regulation_i() const { return get_attr_int("speed_pid/Ki"); }
  auto set_speed_regulation_i(int v) -> decltype(*this) {
    set_attr_int("speed_pid/Ki", v);
    return *this;
  }

  // Speed Regulation D: read/write
  // The derivative constant for the speed regulation PID.
  int speed_regulation_d() const { return get_attr_int("speed_pid/Kd"); }
  auto set_speed_regulation_d(int v) -> decltype(*this) {
    set_attr_int("speed_pid/Kd", v);
    return *this;
  }

  // State: read-only
  // Reading returns a list of state flags. Possible flags are
  // `running`, `ramping` `holding` and `stalled`.
  mode_set state() const { return get_attr_set("state"); }

  // Stop Command: read/write
  // Reading returns the current stop command. Writing sets the stop command.
  // The value determines the motors behavior when `command` is set to `stop`.
  // Also, it determines the motors behavior when a run command completes. See
  // `stop_commands` for a list of possible values.
  std::string stop_command() const { return get_attr_string("stop_command"); }
  auto set_stop_command(std::string v) -> decltype(*this) {
    set_attr_string("stop_command", v);
    return *this;
  }

  // Stop Commands: read-only
  // Returns a list of stop modes supported by the motor controller.
  // Possible values are `coast`, `brake` and `hold`. `coast` means that power will
  // be removed from the motor and it will freely coast to a stop. `brake` means
  // that power will be removed from the motor and a passive electrical load will
  // be placed on the motor. This is usually done by shorting the motor terminals
  // together. This load will absorb the energy from the rotation of the motors and
  // cause the motor to stop more quickly than coasting. `hold` does not remove
  // power from the motor. Instead it actively try to hold the motor at the current
  // position. If an external force tries to turn the motor, the motor will 'push
  // back' to maintain its position.
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


//~autogen

//~autogen motor_commands classes.motor>currentClass

    // Run the motor until another command is sent.
    void run_forever() { set_command("run-forever"); }

    // Run to an absolute position specified by `position_sp` and then
    // stop using the command specified in `stop_command`.
    void run_to_abs_pos() { set_command("run-to-abs-pos"); }

    // Run to a position relative to the current `position` value.
    // The new position will be current `position` + `position_sp`.
    // When the new position is reached, the motor will stop using
    // the command specified by `stop_command`.
    void run_to_rel_pos() { set_command("run-to-rel-pos"); }

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

    // Reset all of the motor parameter attributes to their default value.
    // This will also have the effect of stopping the motor.
    void reset() { set_command("reset"); }


//~autogen

protected:
  motor() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;
};


} //end namespace

#endif //MOTOR_H
