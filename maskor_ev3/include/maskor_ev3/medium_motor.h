#ifndef _MEDIUM_MOTOR_H_
#define _MEDIUM_MOTOR_H_

#include <maskor_ev3/motor.h>

namespace maskor_ev3 {

// EV3 medium motor
class medium_motor : public motor {
public:
  medium_motor(address_type address = OUTPUT_AUTO);
};

}//end namespace

#endif //MEDIUM_MOTOR_H
