#ifndef _BUTTON_H_
#define _BUTTON_H_


// EV3 buttons
class button
{
public:
  button(int bit);

  // Check if the button is pressed.
  bool pressed() const;

  // Gets called whenever the button state changes.
  // The user has to call the process() function to check for state change.
  std::function<void(bool)> onclick;

  // Check if the button state has changed,
  // call onclick function in case it has.
  // Returns true if the state has changed since the last call.
  bool process();

  static button back;
  static button left;
  static button right;
  static button up;
  static button down;
  static button enter;

  // Call process() for each of the EV3 buttons.
  // Returns true if any of the states have changed since the last call.
  static bool process_all();

private:
  int _bit;
  bool _state = false;
  std::vector<unsigned long> _buf;

  struct file_descriptor {
    int _fd;

    file_descriptor(const char *path, int flags);
    ~file_descriptor();
    operator int() { return _fd; }
  };

  std::shared_ptr<file_descriptor> _fd;
};


#endif //BUTTON_H
