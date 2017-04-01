#ifndef _ROS_robotino_msgs_BHAReadings_h
#define _ROS_robotino_msgs_BHAReadings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace robotino_msgs
{

  class BHAReadings : public ros::Msg
  {
    public:
      ros::Time stamp;
      uint8_t pressures_length;
      float st_pressures;
      float * pressures;
      uint8_t cablepull_length;
      float st_cablepull;
      float * cablepull;

    BHAReadings():
      stamp(),
      pressures_length(0), pressures(NULL),
      cablepull_length(0), cablepull(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
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
      *(outbuffer + offset++) = cablepull_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cablepull_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cablepulli;
      u_cablepulli.real = this->cablepull[i];
      *(outbuffer + offset + 0) = (u_cablepulli.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cablepulli.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cablepulli.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cablepulli.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cablepull[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
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
      uint8_t cablepull_lengthT = *(inbuffer + offset++);
      if(cablepull_lengthT > cablepull_length)
        this->cablepull = (float*)realloc(this->cablepull, cablepull_lengthT * sizeof(float));
      offset += 3;
      cablepull_length = cablepull_lengthT;
      for( uint8_t i = 0; i < cablepull_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cablepull;
      u_st_cablepull.base = 0;
      u_st_cablepull.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cablepull.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cablepull.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cablepull.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cablepull = u_st_cablepull.real;
      offset += sizeof(this->st_cablepull);
        memcpy( &(this->cablepull[i]), &(this->st_cablepull), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "robotino_msgs/BHAReadings"; };
    const char * getMD5(){ return "db28e0e5f4b0a0da881baf9c7f3c94a1"; };

  };

}
#endif