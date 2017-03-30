#ifndef _LARGE_MOTOR_H_
#define _LARGE_MOTOR_H_

#include <maskor_ev3/motor.h>

namespace maskor_ev3 {

// EV3 large motor
class large_motor : public motor
{
public:
  large_motor(address_type address = OUTPUT_AUTO);
};

}//end namespace

#endif //LARGE_MOTOR_H
