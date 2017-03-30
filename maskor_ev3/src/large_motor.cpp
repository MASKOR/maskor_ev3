#include <maskor_ev3/large_motor.h>

namespace maskor_ev3 {

large_motor::large_motor(address_type address) : motor(address, motor_large)
{
}

}
