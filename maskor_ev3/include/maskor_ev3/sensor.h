#ifndef _SENSOR_H_
#define _SENSOR_H_

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
// `driver_name`. Then it will not matter which port a sensor is plugged in to - your
// program will still work.

class sensor : protected device
{
public:
  typedef device_type sensor_type;

  static const sensor_type ev3_touch;
  static const sensor_type ev3_color;
  static const sensor_type ev3_ultrasonic;
  static const sensor_type ev3_gyro;
  static const sensor_type ev3_infrared;

  static const sensor_type nxt_touch;
  static const sensor_type nxt_light;
  static const sensor_type nxt_sound;
  static const sensor_type nxt_ultrasonic;
  static const sensor_type nxt_i2c_sensor;
  static const sensor_type nxt_analog;

  sensor(address_type);
  sensor(address_type, const std::set<sensor_type>&);

  using device::connected;
  using device::device_index;

  // Returns the value or values measured by the sensor. Check `num_values` to
  // see how many values there are. Values with index >= num_values will return
  // an error. The values are fixed point numbers, so check `decimals` to see
  // if you need to divide to get the actual value.
  int   value(unsigned index=0) const;

  // The value converted to float using `decimals`.
  float float_value(unsigned index=0) const;

  // Human-readable name of the connected sensor.
  std::string type_name() const;

  // Bin Data Format: read-only
  // Returns the format of the values in `bin_data` for the current mode.
  // Possible values are:
  //
  //    - `u8`: Unsigned 8-bit integer (byte)
  //    - `s8`: Signed 8-bit integer (sbyte)
  //    - `u16`: Unsigned 16-bit integer (ushort)
  //    - `s16`: Signed 16-bit integer (short)
  //    - `s16_be`: Signed 16-bit integer, big endian
  //    - `s32`: Signed 32-bit integer (int)
  //    - `float`: IEEE 754 32-bit floating point (float)
  std::string bin_data_format() const { return get_attr_string("bin_data_format"); };

  // Bin Data: read-only
  // Returns the unscaled raw values in the `value<N>` attributes as raw byte
  // array. Use `bin_data_format`, `num_values` and the individual sensor
  // documentation to determine how to interpret the data.
  const std::vector<char>& bin_data() const;

  // Bin Data: read-only
  // Writes the unscaled raw values in the `value<N>` attributes into the
  // user-provided struct/buffer.  Use `bin_data_format`, `num_values` and the
  // individual sensor documentation to determine how to interpret the data.
  template <class T>
  void bin_data(T *buf) const {
      bin_data(); // fills _bin_data
      std::copy_n(_bin_data.data(), _bin_data.size(), static_cast<char*>(buf));
  }

//~autogen generic-get-set classes.sensor>currentClass

  // Command: write-only
  // Sends a command to the sensor.
  auto set_command(std::string v) -> decltype(*this) {
    set_attr_string("command", v);
    return *this;
  }

  // Commands: read-only
  // Returns a list of the valid commands for the sensor.
  // Returns -EOPNOTSUPP if no commands are supported.
  mode_set commands() const { return get_attr_set("commands"); }

  // Decimals: read-only
  // Returns the number of decimal places for the values in the `value<N>`
  // attributes of the current mode.
  int decimals() const { return get_attr_int("decimals"); }

  // Driver Name: read-only
  // Returns the name of the sensor device/driver. See the list of [supported
  // sensors] for a complete list of drivers.
  std::string driver_name() const { return get_attr_string("driver_name"); }

  // Mode: read/write
  // Returns the current mode. Writing one of the values returned by `modes`
  // sets the sensor to that mode.
  std::string mode() const { return get_attr_string("mode"); }
  auto set_mode(std::string v) -> decltype(*this) {
    set_attr_string("mode", v);
    return *this;
  }

  // Modes: read-only
  // Returns a list of the valid modes for the sensor.
  mode_set modes() const { return get_attr_set("modes"); }

  // Num Values: read-only
  // Returns the number of `value<N>` attributes that will return a valid value
  // for the current mode.
  int num_values() const { return get_attr_int("num_values"); }

  // Address: read-only
  // Returns the name of the port that the sensor is connected to, e.g. `ev3:in1`.
  // I2C sensors also include the I2C address (decimal), e.g. `ev3:in1:i2c8`.
  std::string address() const { return get_attr_string("address"); }

  // Units: read-only
  // Returns the units of the measured value for the current mode. May return
  // empty string
  std::string units() const { return get_attr_string("units"); }


//~autogen

protected:
  sensor() {}

  bool connect(const std::map<std::string, std::set<std::string>>&) noexcept;

  mutable std::vector<char> _bin_data;
};


#endif //SENSOR_H
