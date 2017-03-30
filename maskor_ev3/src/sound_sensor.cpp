#include <maskor_ev3/sound_sensor.h>
#include <maskor_ev3/port.h>

namespace maskor_ev3 {

sound_sensor::sound_sensor(address_type address) :
  sensor(address, { nxt_sound, nxt_analog })
{
    if (connected() && driver_name() == nxt_analog) {
        lego_port port(address);

        if (port.connected()) {
            port.set_set_device(nxt_sound);

            if (port.status() != nxt_sound) {
                // Failed to load lego-nxt-sound friver. Wrong port?
                _path.clear();
            }
        } else {
            _path.clear();
        }
    }
}

}//end namespace
