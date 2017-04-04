#ifndef _LCD_H_
#define _LCD_H_

/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 * Copyright (c) 2014 - Franz Detro
*/

#include <stdint.h>

namespace maskor_ev3 {

// EV3 LCD
class lcd
{
public:
  lcd();
  ~lcd();

  bool available() const { return _fb != nullptr; }

  uint32_t resolution_x()   const { return _xres; }
  uint32_t resolution_y()   const { return _yres; }
  uint32_t bits_per_pixel() const { return _bpp; }

  uint32_t frame_buffer_size() const { return _fbsize; }
  uint32_t line_length()       const { return _llength; }

  unsigned char *frame_buffer() { return _fb; }

  void fill(unsigned char pixel);

protected:
  void init();
  void deinit();

private:
  unsigned char *_fb;
  uint32_t _fbsize;
  uint32_t _llength;
  uint32_t _xres;
  uint32_t _yres;
  uint32_t _bpp;
};

}//end namespace


#endif //LCD_H
