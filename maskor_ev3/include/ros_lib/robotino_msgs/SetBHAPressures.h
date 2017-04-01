#ifndef _ROS_robotino_msgs_SetBHAPressures_h
#define _ROS_robotino_msgs_SetBHAPressures_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotino_msgs
{

  class SetBHAPressures : public ros::Msg
  {
    public:
      uint8_t pressures_length;
      float st_pressures;
      float * pressures;

    SetBHAPressures():
      pressures_length(0), pressures(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = pressures_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < pressures_length; i++){
      union {
        float real;
        uint32_t base;
      } u_pressuresi;
      u_pressuresi.real = this->pressures[i];
      *(outbuffer + offset + 0) = (u_pressuresi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pressuresi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pressuresi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pressuresi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pressures[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t pressures_lengthT = *(inbuffer + offset++);
      if(pressures_lengthT > pressures_length)
        this->pressures = (float*)realloc(this->pressures, pressures_lengthT * sizeof(float));
      offset += 3;
      pressures_length = pressures_lengthT;
      for( uint8_t i = 0; i < pressures_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_pressures;
      u_st_pressures.base = 0;
      u_st_pressures.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_pressures.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_pressures.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_pressures.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_pressures = u_st_pressures.real;
      offset += sizeof(this->st_pressures);
        memcpy( &(this->pressures[i]), &(this->st_pressures), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "robotino_msgs/SetBHAPressures"; };
    const char * getMD5(){ return "f07803ec936ff4605f313dd88545e5bb"; };

  };

}
#endif