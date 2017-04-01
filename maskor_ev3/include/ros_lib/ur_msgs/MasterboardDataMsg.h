#ifndef _ROS_ur_msgs_MasterboardDataMsg_h
#define _ROS_ur_msgs_MasterboardDataMsg_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace ur_msgs
{

  class MasterboardDataMsg : public ros::Msg
  {
    public:
      int16_t digital_input_bits;
      int16_t digital_output_bits;
      int8_t analog_input_range0;
      int8_t analog_input_range1;
      double analog_input0;
      double analog_input1;
      int8_t analog_output_domain0;
      int8_t analog_output_domain1;
      double analog_output0;
      double analog_output1;
      float masterboard_temperature;
      float robot_voltage_48V;
      float robot_current;
      float master_io_current;
      uint8_t master_safety_state;
      uint8_t master_onoff_state;

    MasterboardDataMsg():
      digital_input_bits(0),
      digital_output_bits(0),
      analog_input_range0(0),
      analog_input_range1(0),
      analog_input0(0),
      analog_input1(0),
      analog_output_domain0(0),
      analog_output_domain1(0),
      analog_output0(0),
      analog_output1(0),
      masterboard_temperature(0),
      robot_voltage_48V(0),
      robot_current(0),
      master_io_current(0),
      master_safety_state(0),
      master_onoff_state(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_digital_input_bits;
      u_digital_input_bits.real = this->digital_input_bits;
      *(outbuffer + offset + 0) = (u_digital_input_bits.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_digital_input_bits.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->digital_input_bits);
      union {
        int16_t real;
        uint16_t base;
      } u_digital_output_bits;
      u_digital_output_bits.real = this->digital_output_bits;
      *(outbuffer + offset + 0) = (u_digital_output_bits.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_digital_output_bits.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->digital_output_bits);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_input_range0;
      u_analog_input_range0.real = this->analog_input_range0;
      *(outbuffer + offset + 0) = (u_analog_input_range0.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->analog_input_range0);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_input_range1;
      u_analog_input_range1.real = this->analog_input_range1;
      *(outbuffer + offset + 0) = (u_analog_input_range1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->analog_input_range1);
      union {
        double real;
        uint64_t base;
      } u_analog_input0;
      u_analog_input0.real = this->analog_input0;
      *(outbuffer + offset + 0) = (u_analog_input0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_analog_input0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_analog_input0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_analog_input0.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_analog_input0.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_analog_input0.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_analog_input0.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_analog_input0.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->analog_input0);
      union {
        double real;
        uint64_t base;
      } u_analog_input1;
      u_analog_input1.real = this->analog_input1;
      *(outbuffer + offset + 0) = (u_analog_input1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_analog_input1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_analog_input1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_analog_input1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_analog_input1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_analog_input1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_analog_input1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_analog_input1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->analog_input1);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_output_domain0;
      u_analog_output_domain0.real = this->analog_output_domain0;
      *(outbuffer + offset + 0) = (u_analog_output_domain0.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->analog_output_domain0);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_output_domain1;
      u_analog_output_domain1.real = this->analog_output_domain1;
      *(outbuffer + offset + 0) = (u_analog_output_domain1.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->analog_output_domain1);
      union {
        double real;
        uint64_t base;
      } u_analog_output0;
      u_analog_output0.real = this->analog_output0;
      *(outbuffer + offset + 0) = (u_analog_output0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_analog_output0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_analog_output0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_analog_output0.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_analog_output0.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_analog_output0.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_analog_output0.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_analog_output0.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->analog_output0);
      union {
        double real;
        uint64_t base;
      } u_analog_output1;
      u_analog_output1.real = this->analog_output1;
      *(outbuffer + offset + 0) = (u_analog_output1.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_analog_output1.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_analog_output1.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_analog_output1.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_analog_output1.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_analog_output1.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_analog_output1.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_analog_output1.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->analog_output1);
      union {
        float real;
        uint32_t base;
      } u_masterboard_temperature;
      u_masterboard_temperature.real = this->masterboard_temperature;
      *(outbuffer + offset + 0) = (u_masterboard_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_masterboard_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_masterboard_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_masterboard_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->masterboard_temperature);
      union {
        float real;
        uint32_t base;
      } u_robot_voltage_48V;
      u_robot_voltage_48V.real = this->robot_voltage_48V;
      *(outbuffer + offset + 0) = (u_robot_voltage_48V.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_robot_voltage_48V.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_robot_voltage_48V.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_robot_voltage_48V.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->robot_voltage_48V);
      union {
        float real;
        uint32_t base;
      } u_robot_current;
      u_robot_current.real = this->robot_current;
      *(outbuffer + offset + 0) = (u_robot_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_robot_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_robot_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_robot_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->robot_current);
      union {
        float real;
        uint32_t base;
      } u_master_io_current;
      u_master_io_current.real = this->master_io_current;
      *(outbuffer + offset + 0) = (u_master_io_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_master_io_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_master_io_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_master_io_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->master_io_current);
      *(outbuffer + offset + 0) = (this->master_safety_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->master_safety_state);
      *(outbuffer + offset + 0) = (this->master_onoff_state >> (8 * 0)) & 0xFF;
      offset += sizeof(this->master_onoff_state);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_digital_input_bits;
      u_digital_input_bits.base = 0;
      u_digital_input_bits.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_digital_input_bits.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->digital_input_bits = u_digital_input_bits.real;
      offset += sizeof(this->digital_input_bits);
      union {
        int16_t real;
        uint16_t base;
      } u_digital_output_bits;
      u_digital_output_bits.base = 0;
      u_digital_output_bits.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_digital_output_bits.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->digital_output_bits = u_digital_output_bits.real;
      offset += sizeof(this->digital_output_bits);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_input_range0;
      u_analog_input_range0.base = 0;
      u_analog_input_range0.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->analog_input_range0 = u_analog_input_range0.real;
      offset += sizeof(this->analog_input_range0);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_input_range1;
      u_analog_input_range1.base = 0;
      u_analog_input_range1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->analog_input_range1 = u_analog_input_range1.real;
      offset += sizeof(this->analog_input_range1);
      union {
        double real;
        uint64_t base;
      } u_analog_input0;
      u_analog_input0.base = 0;
      u_analog_input0.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_analog_input0.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_analog_input0.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_analog_input0.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_analog_input0.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_analog_input0.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_analog_input0.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_analog_input0.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->analog_input0 = u_analog_input0.real;
      offset += sizeof(this->analog_input0);
      union {
        double real;
        uint64_t base;
      } u_analog_input1;
      u_analog_input1.base = 0;
      u_analog_input1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_analog_input1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_analog_input1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_analog_input1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_analog_input1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_analog_input1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_analog_input1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_analog_input1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->analog_input1 = u_analog_input1.real;
      offset += sizeof(this->analog_input1);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_output_domain0;
      u_analog_output_domain0.base = 0;
      u_analog_output_domain0.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->analog_output_domain0 = u_analog_output_domain0.real;
      offset += sizeof(this->analog_output_domain0);
      union {
        int8_t real;
        uint8_t base;
      } u_analog_output_domain1;
      u_analog_output_domain1.base = 0;
      u_analog_output_domain1.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->analog_output_domain1 = u_analog_output_domain1.real;
      offset += sizeof(this->analog_output_domain1);
      union {
        double real;
        uint64_t base;
      } u_analog_output0;
      u_analog_output0.base = 0;
      u_analog_output0.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_analog_output0.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_analog_output0.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_analog_output0.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_analog_output0.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_analog_output0.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_analog_output0.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_analog_output0.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->analog_output0 = u_analog_output0.real;
      offset += sizeof(this->analog_output0);
      union {
        double real;
        uint64_t base;
      } u_analog_output1;
      u_analog_output1.base = 0;
      u_analog_output1.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_analog_output1.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_analog_output1.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_analog_output1.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_analog_output1.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_analog_output1.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_analog_output1.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_analog_output1.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->analog_output1 = u_analog_output1.real;
      offset += sizeof(this->analog_output1);
      union {
        float real;
        uint32_t base;
      } u_masterboard_temperature;
      u_masterboard_temperature.base = 0;
      u_masterboard_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_masterboard_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_masterboard_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_masterboard_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->masterboard_temperature = u_masterboard_temperature.real;
      offset += sizeof(this->masterboard_temperature);
      union {
        float real;
        uint32_t base;
      } u_robot_voltage_48V;
      u_robot_voltage_48V.base = 0;
      u_robot_voltage_48V.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_robot_voltage_48V.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_robot_voltage_48V.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_robot_voltage_48V.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->robot_voltage_48V = u_robot_voltage_48V.real;
      offset += sizeof(this->robot_voltage_48V);
      union {
        float real;
        uint32_t base;
      } u_robot_current;
      u_robot_current.base = 0;
      u_robot_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_robot_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_robot_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_robot_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->robot_current = u_robot_current.real;
      offset += sizeof(this->robot_current);
      union {
        float real;
        uint32_t base;
      } u_master_io_current;
      u_master_io_current.base = 0;
      u_master_io_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_master_io_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_master_io_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_master_io_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->master_io_current = u_master_io_current.real;
      offset += sizeof(this->master_io_current);
      this->master_safety_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->master_safety_state);
      this->master_onoff_state =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->master_onoff_state);
     return offset;
    }

    const char * getType(){ return "ur_msgs/MasterboardDataMsg"; };
    const char * getMD5(){ return "a4aa4d8ccbd10a18ef4008b679f6ccbe"; };

  };

}
#endif