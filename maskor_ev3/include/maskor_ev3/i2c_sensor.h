#ifndef _I2C_SENSOR_H_
#define _I2C_SENSOR_H_

class i2c_sensor : public sensor
{
public:
  i2c_sensor(address_type address = INPUT_AUTO);

//~autogen generic-get-set classes.i2cSensor>currentClass

  // FW Version: read-only
  // Returns the firmware version of the sensor if available. Currently only
  // I2C/NXT sensors support this.
  std::string fw_version() const { return get_attr_string("fw_version"); }

  // Poll MS: read/write
  // Returns the polling period of the sensor in milliseconds. Writing sets the
  // polling period. Setting to 0 disables polling. Minimum value is hard
  // coded as 50 msec. Returns -EOPNOTSUPP if changing polling is not supported.
  // Currently only I2C/NXT sensors support changing the polling period.
  int poll_ms() const { return get_attr_int("poll_ms"); }
  auto set_poll_ms(int v) -> decltype(*this) {
    set_attr_int("poll_ms", v);
    return *this;
  }
};


#endif // I2C_SENSOR_H
