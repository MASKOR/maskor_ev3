#include <maskor_ev3/device.h>
//include <maskor_ev3/stream.h>

namespace maskor_ev3 {

bool device::connect(
    const std::string &dir,
    const std::string &pattern,
    const std::map<std::string, std::set<std::string>> &match
    ) noexcept
{
  using namespace std;

  const size_t pattern_length = pattern.length();

  struct dirent *dp;
  DIR *dfd;

  if ((dfd = opendir(dir.c_str())) != nullptr)
  {
    while ((dp = readdir(dfd)) != nullptr)
    {
      if (strncmp(dp->d_name, pattern.c_str(), pattern_length)==0)
      {
        try
        {
          _path = dir + dp->d_name + '/';

          bool bMatch = true;
          for (auto &m : match)
          {
            const auto &attribute = m.first;
            const auto &matches   = m.second;
            const auto strValue   = get_attr_string(attribute);

            if (!matches.empty() && !matches.begin()->empty() &&
                (matches.find(strValue) == matches.end()))
            {
              bMatch = false;
              break;
            }
          }

          if (bMatch) {
            closedir(dfd);
            return true;
          }
        }
        catch (...) { }

        _path.clear();
      }
    }

    closedir(dfd);
  }

  return false;
}

//-----------------------------------------------------------------------------

int device::device_index() const
{
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  if (_device_index < 0)
  {
    unsigned f = 1;
    _device_index = 0;
    for (auto it=_path.rbegin(); it!=_path.rend(); ++it)
    {
      if ((*it < '0') || (*it > '9'))
        break;

      _device_index += (*it -'0') * f;
      f *= 10;
    }
  }

  return _device_index;
}

//-----------------------------------------------------------------------------

int device::get_attr_int(const std::string &name) const {
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  for(int attempt = 0; attempt < 2; ++attempt) {
    ifstream &is = ifstream_open(_path + name);
    if (is.is_open())
    {
      int result = 0;
      try {
        is >> result;
        return result;
      } catch(...) {
        // This could mean the sysfs attribute was recreated and the
        // corresponding file handle got stale. Lets close the file and try
        // again (once):
        if (attempt != 0) throw;

        is.close();
        is.clear();
      }
    } else break;
  }
  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

void device::set_attr_int(const std::string &name, int value) {
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  for(int attempt = 0; attempt < 2; ++attempt) {
    ofstream &os = ofstream_open(_path + name);
    if (os.is_open())
    {
      if (os << value) return;

      // An error could mean that sysfs attribute was recreated and the cached
      // file handle is stale. Lets close the file and try again (once):
      if (attempt == 0 && errno == ENODEV) {
        os.close();
        os.clear();
      } else {
        throw system_error(std::error_code(errno, std::system_category()));
      }
    } else {
      throw system_error(make_error_code(errc::no_such_device), _path + name);
    }
  }
}

//-----------------------------------------------------------------------------

std::string device::get_attr_string(const std::string &name) const
{
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  ifstream &is = ifstream_open(_path + name);
  if (is.is_open())
  {
    string result;
    is >> result;
    return result;
  }

  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

void device::set_attr_string(const std::string &name, const std::string &value)
{
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  ofstream &os = ofstream_open(_path + name);
  if (os.is_open())
  {
    if (!(os << value)) throw system_error(std::error_code(errno, std::system_category()));
    return;
  }

  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

std::string device::get_attr_line(const std::string &name) const
{
  using namespace std;

  if (_path.empty())
    throw system_error(make_error_code(errc::function_not_supported), "no device connected");

  ifstream &is = ifstream_open(_path + name);
  if (is.is_open())
  {
    string result;
    getline(is, result);
    return result;
  }

  throw system_error(make_error_code(errc::no_such_device), _path+name);
}

//-----------------------------------------------------------------------------

mode_set device::get_attr_set(const std::string &name,
                              std::string *pCur) const
{
  using namespace std;

  string s = get_attr_line(name);

  mode_set result;
  size_t pos, last_pos = 0;
  string t;
  do {
    pos = s.find(' ', last_pos);

    if (pos != string::npos)
    {
      t = s.substr(last_pos, pos-last_pos);
      last_pos = pos+1;
    }
    else
      t = s.substr(last_pos);

    if (!t.empty())
    {
      if (*t.begin()=='[')
      {
        t = t.substr(1, t.length()-2);
        if (pCur)
          *pCur = t;
      }
      result.insert(t);
    }
  } while (pos!=string::npos);

  return result;
}

//-----------------------------------------------------------------------------

std::string device::get_attr_from_set(const std::string &name) const
{
  using namespace std;

  string s = get_attr_line(name);

  size_t pos, last_pos = 0;
  string t;
  do {
    pos = s.find(' ', last_pos);

    if (pos != string::npos)
    {
      t = s.substr(last_pos, pos-last_pos);
      last_pos = pos+1;
    }
    else
      t = s.substr(last_pos);

    if (!t.empty())
    {
      if (*t.begin()=='[')
      {
        return t.substr(1, t.length()-2);
      }
    }
  } while (pos!=string::npos);

  return { "none" };
}


} //end namespace
