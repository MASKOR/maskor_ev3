#include <maskor_ev3/button.h>


namespace maskor_ev3 {

button::file_descriptor::file_descriptor(const char *path, int flags)
  : _fd(open(path, flags))
{}

button::file_descriptor::~file_descriptor()
{
  if (_fd != -1) close(_fd);
}

//-----------------------------------------------------------------------------

button::button(int bit)
  : _bit(bit),
    _buf((KEY_CNT + bits_per_long - 1) / bits_per_long),
    _fd( new file_descriptor("/dev/input/by-path/platform-gpio-keys.0-event", O_RDONLY) )
{ }

//-----------------------------------------------------------------------------

bool button::pressed() const
{
#ifndef NO_LINUX_HEADERS 
  if (ioctl(*_fd, EVIOCGKEY(_buf.size()), _buf.data()) < 0)
  {
    // handle error
  }
#endif
  // bit in bytes is 1 when released and 0 when pressed
  return !(_buf[_bit / bits_per_long] & 1 << (_bit % bits_per_long));
}

//-----------------------------------------------------------------------------

bool button::process()
{
  bool new_state = pressed();

  if (new_state != _state) {
    _state = new_state;
    if (onclick) onclick(new_state);
    return true;
  }

  return false;
}

//-----------------------------------------------------------------------------

#ifndef NO_LINUX_HEADERS
button button::back (KEY_BACKSPACE);
button button::left (KEY_LEFT);
button button::right(KEY_RIGHT);
button button::up   (KEY_UP);
button button::down (KEY_DOWN);
button button::enter(KEY_ENTER);
#endif

//-----------------------------------------------------------------------------

bool button::process_all() {
  std::array<bool, 6> changed{{
    back. process(),
    left. process(),
    right.process(),
    up.   process(),
    down. process(),
    enter.process()
  }};
  return std::any_of(changed.begin(), changed.end(), [](bool c){ return c; });
}

}//end namespace
