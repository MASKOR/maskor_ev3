#ifndef _ROS_maskor_ev3_msgs_GyroSensor_h
#define _ROS_maskor_ev3_msgs_GyroSensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace maskor_ev3_msgs
{

  class GyroSensor : public ros::Msg
  {
    public:
      std_msgs::Header header;
      int16_t angle;
      int16_t rotational_speed;

    GyroSensor():
      header(),
      angle(0),
      rotational_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_angle;
      u_angle.real = this->angle;
      *(outbuffer + offset + 0) = (u_angle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->angle);
      union {
        int16_t real;
        uint16_t base;
      } u_rotational_speed;
      u_rotational_speed.real = this->rotational_speed;
      *(outbuffer + offset + 0) = (u_rotational_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_rotational_speed.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rotational_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_angle;
      u_angle.base = 0;
      u_angle.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->angle = u_angle.real;
      offset += sizeof(this->angle);
      union {
        int16_t real;
        uint16_t base;
      } u_rotational_speed;
      u_rotational_speed.base = 0;
      u_rotational_speed.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_rotational_speed.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->rotational_speed = u_rotational_speed.real;
      offset += sizeof(this->rotational_speed);
     return offset;
    }

    const char * getType(){ return "maskor_ev3_msgs/GyroSensor"; };
    const char * getMD5(){ return "82b4bd310d13cacb5a99b49929d4a59a"; };

  };

}
#endif