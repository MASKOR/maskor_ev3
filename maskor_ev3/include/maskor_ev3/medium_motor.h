#ifndef _MEDIUM_MOTOR_H_
#define _MEDIUM_MOTOR_H_

/*
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 * Copyright (c) 2014 - Franz Detro
*/


#include <maskor_ev3/motor.h>

namespace maskor_ev3 {

// EV3 medium motor
class medium_motor : public motor {
public:
  medium_motor(address_type address = OUTPUT_AUTO);
};

}//end namespace

#endif //MEDIUM_MOTOR_H
