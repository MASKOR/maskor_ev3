#include <maskor_ev3/medium_motor.h>

namespace maskor_ev3 {

medium_motor::medium_motor(address_type address) : motor(address, motor_medium)
{
}

}
