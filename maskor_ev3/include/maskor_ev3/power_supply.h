#ifndef _POWER_SUPPLY_H_
#define _POWER_SUPPLY_H_

class power_supply : protected device
{
public:
  power_supply(std::string name);

  using device::connected;

//~autogen generic-get-set classes.powerSupply>currentClass

  // Measured Current: read-only
  // The measured current that the battery is supplying (in microamps)
  int measured_current() const { return get_attr_int("current_now"); }

  // Measured Voltage: read-only
  // The measured voltage that the battery is supplying (in microvolts)
  int measured_voltage() const { return get_attr_int("voltage_now"); }

  // Max Voltage: read-only
  int max_voltage() const { return get_attr_int("voltage_max_design"); }

  // Min Voltage: read-only
  int min_voltage() const { return get_attr_int("voltage_min_design"); }

  // Technology: read-only
  std::string technology() const { return get_attr_string("technology"); }

  // Type: read-only
  std::string type() const { return get_attr_string("type"); }


//~autogen

  float measured_amps()       const { return measured_current() / 1000000.f; }
  float measured_volts()      const { return measured_voltage() / 1000000.f; }

  static power_supply battery;
};

#endif //POWER_SUPPLY_H
