#include <maskor_ev3/sensor.h>

namespace maskor_ev3 {

  sensor::sensor(address_type address)
  {
    connect({{ "address", { address }}});
  }

  //-----------------------------------------------------------------------------

  sensor::sensor(address_type address, const std::set<sensor_type> &types)
  {
    connect({{ "address", { address }},
	  { "driver_name", types }});
  }

  //-----------------------------------------------------------------------------

  bool sensor::connect(const std::map<std::string, std::set<std::string>> &match) noexcept
  {
    static const std::string _strClassDir { SYS_ROOT "/lego-sensor/" };
    static const std::string _strPattern  { "sensor" };

    try
      {
	if (device::connect(_strClassDir, _strPattern, match))
	  {
	    return true;
	  }
      }
    catch (...) { }

    _path.clear();

    return false;
  }

  //-----------------------------------------------------------------------------

  std::string sensor::type_name() const
  {
    auto type = driver_name();
    if (type.empty())
      {
	static const std::string s("<none>");
	return s;
      }

    static const std::map<sensor_type, const std::string> lookup_table {
      { ev3_touch,       "EV3 touch" },
	{ ev3_color,       "EV3 color" },
	  { ev3_ultrasonic,  "EV3 ultrasonic" },
	    { ev3_gyro,        "EV3 gyro" },
	      { ev3_infrared,    "EV3 infrared" },
		{ nxt_touch,       "NXT touch" },
		  { nxt_light,       "NXT light" },
		    { nxt_sound,       "NXT sound" },
		      { nxt_ultrasonic,  "NXT ultrasonic" },
			{ nxt_i2c_sensor,  "I2C sensor" },
			  };

    auto s = lookup_table.find(type);
    if (s != lookup_table.end())
      return s->second;

    return type;
  }

  //-----------------------------------------------------------------------------

  int sensor::value(unsigned index) const
  {
    if (static_cast<int>(index) >= num_values())
      throw std::invalid_argument("index");

    char svalue[7] = "value0";
    svalue[5] += index;

    return get_attr_int(svalue);
  }

  //-----------------------------------------------------------------------------

  float sensor::float_value(unsigned index) const
  {
    return value(index) * powf(10, -decimals());
  }

  //-----------------------------------------------------------------------------
  const std::vector<char>& sensor::bin_data() const
  {
    using namespace std;

    if (_path.empty())
      throw system_error(make_error_code(errc::function_not_supported), "no device connected");

    if (_bin_data.empty()) {
      static const map<string, int> lookup_table {
	{"u8",     1},
	  {"s8",     1},
	    {"u16",    2},
	      {"s16",    2},
		{"s16_be", 2},
		  {"s32",    4},
		    {"float",  4}
      };

      int value_size = 1;

      auto s = lookup_table.find(bin_data_format());
      if (s != lookup_table.end())
	value_size = s->second;

      _bin_data.resize(num_values() * value_size);
    }

    const string fname = _path + "bin_data";

    ifstream &is = ifstream_open(fname);
    if (is.is_open())
      {
    	is.read(_bin_data.data(), _bin_data.size());
    	return _bin_data;
      }

    throw system_error(make_error_code(errc::no_such_device), fname);
  }

} //end namespace
