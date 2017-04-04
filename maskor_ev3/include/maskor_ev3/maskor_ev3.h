#ifndef _MASKOR_EV3_H_
#define _MASKOR_EV3_H_
/*
 * MASKOR LEGO EV3 library
 *
 * Copyright (c) 2017 
 * Marcel Stüttgen 
 * stuettgen@fh-aachen.de
 * https://www.maskor.fh-aachen.de
 *
 * based on and refactored from "ev3-dev" package, the
 * C++ API to the sensors, motors, buttons, LEDs and battery of the ev3dev
 * Linux kernel for the LEGO Mindstorms EV3 hardware
 * Copyright (c) 2014 - Franz Detro
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#include <maskor_ev3/common.h>
#include <maskor_ev3/device.h>
#include <maskor_ev3/sensor.h>
#include <maskor_ev3/i2c_sensor.h>
#include <maskor_ev3/touch_sensor.h>
#include <maskor_ev3/color_sensor.h>
#include <maskor_ev3/infrared_sensor.h>
#include <maskor_ev3/gyro_sensor.h>
#include <maskor_ev3/ultrasonic_sensor.h>
#include <maskor_ev3/sound_sensor.h>
#include <maskor_ev3/light_sensor.h>
#include <maskor_ev3/motor.h>
#include <maskor_ev3/medium_motor.h>
#include <maskor_ev3/large_motor.h>
#include <maskor_ev3/dc_motor.h>
#include <maskor_ev3/led.h>
#include <maskor_ev3/power_supply.h>
#include <maskor_ev3/button.h>
#include <maskor_ev3/sound.h>
#include <maskor_ev3/lcd.h>
#include <maskor_ev3/remote_control.h>

namespace maskor_ev3 {

const motor::motor_type motor::motor_large  { "lego-ev3-l-motor" };
const motor::motor_type motor::motor_medium { "lego-ev3-m-motor" };

const sensor::sensor_type sensor::ev3_touch       { "lego-ev3-touch" };
const sensor::sensor_type sensor::ev3_color       { "lego-ev3-color" };
const sensor::sensor_type sensor::ev3_ultrasonic  { "lego-ev3-us" };
const sensor::sensor_type sensor::ev3_gyro        { "lego-ev3-gyro" };
const sensor::sensor_type sensor::ev3_infrared    { "lego-ev3-ir" };

const sensor::sensor_type sensor::nxt_touch       { "lego-nxt-touch" };
const sensor::sensor_type sensor::nxt_light       { "lego-nxt-light" };
const sensor::sensor_type sensor::nxt_sound       { "lego-nxt-sound" };
const sensor::sensor_type sensor::nxt_ultrasonic  { "lego-nxt-us" };
const sensor::sensor_type sensor::nxt_i2c_sensor  { "nxt-i2c-sensor" };
const sensor::sensor_type sensor::nxt_analog      { "nxt-analog" };



// STRINGS
const std::string touch_sensor::mode_touch{ "TOUCH" };

const std::string infrared_sensor::mode_ir_prox{ "IR-PROX" };
const std::string infrared_sensor::mode_ir_seek{ "IR-SEEK" };
const std::string infrared_sensor::mode_ir_remote{ "IR-REMOTE" };
const std::string infrared_sensor::mode_ir_rem_a{ "IR-REM-A" };
const std::string infrared_sensor::mode_ir_cal{ "IR-CAL" };

const std::string color_sensor::mode_col_reflect{ "COL-REFLECT" };
const std::string color_sensor::mode_col_ambient{ "COL-AMBIENT" };
const std::string color_sensor::mode_col_color{ "COL-COLOR" };
const std::string color_sensor::mode_ref_raw{ "REF-RAW" };
const std::string color_sensor::mode_rgb_raw{ "RGB-RAW" };

const std::string ultrasonic_sensor::mode_us_dist_cm{ "US-DIST-CM" };
const std::string ultrasonic_sensor::mode_us_dist_in{ "US-DIST-IN" };
const std::string ultrasonic_sensor::mode_us_listen{ "US-LISTEN" };
const std::string ultrasonic_sensor::mode_us_si_cm{ "US-SI-CM" };
const std::string ultrasonic_sensor::mode_us_si_in{ "US-SI-IN" };

const std::string gyro_sensor::mode_gyro_ang{ "GYRO-ANG" };
const std::string gyro_sensor::mode_gyro_rate{ "GYRO-RATE" };
const std::string gyro_sensor::mode_gyro_fas{ "GYRO-FAS" };
const std::string gyro_sensor::mode_gyro_g_a{ "GYRO-G&A" };
const std::string gyro_sensor::mode_gyro_cal{ "GYRO-CAL" };

const std::string motor::command_run_forever{ "run-forever" };
const std::string motor::command_run_to_abs_pos{ "run-to-abs-pos" };
const std::string motor::command_run_to_rel_pos{ "run-to-rel-pos" };
const std::string motor::command_run_timed{ "run-timed" };
const std::string motor::command_run_direct{ "run-direct" };
const std::string motor::command_stop{ "stop" };
const std::string motor::command_reset{ "reset" };
const std::string motor::encoder_polarity_normal{ "normal" };
const std::string motor::encoder_polarity_inversed{ "inversed" };
const std::string motor::polarity_normal{ "normal" };
const std::string motor::polarity_inversed{ "inversed" };
const std::string motor::speed_regulation_on{ "on" };
const std::string motor::speed_regulation_off{ "off" };
const std::string motor::stop_command_coast{ "coast" };
const std::string motor::stop_command_brake{ "brake" };
const std::string motor::stop_command_hold{ "hold" };

const std::string sound_sensor::mode_db{ "DB" };
const std::string sound_sensor::mode_dba{ "DBA" };

const std::string light_sensor::mode_reflect{ "REFLECT" };
const std::string light_sensor::mode_ambient{ "AMBIENT" };

const std::string dc_motor::command_run_forever{ "run-forever" };
const std::string dc_motor::command_run_timed{ "run-timed" };
const std::string dc_motor::command_run_direct{ "run-direct" };
const std::string dc_motor::command_stop{ "stop" };
const std::string dc_motor::polarity_normal{ "normal" };
const std::string dc_motor::polarity_inversed{ "inversed" };
const std::string dc_motor::stop_command_coast{ "coast" };
const std::string dc_motor::stop_command_brake{ "brake" };

/*
const std::string servo_motor::command_run{ "run" };
const std::string servo_motor::command_float{ "float" };
const std::string servo_motor::polarity_normal{ "normal" };
const std::string servo_motor::polarity_inversed{ "inversed" };
*/

} //end namespace

#endif //MASKOR_EV3_H

