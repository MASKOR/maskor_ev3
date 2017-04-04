#ifndef _ROS_robotino_msgs_AnalogReadings_h
#define _ROS_robotino_msgs_AnalogReadings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace robotino_msgs
{

  class AnalogReadings : public ros::Msg
  {
    public:
      ros::Time stamp;
      uint8_t values_length;
      float st_values;
      float * values;

    AnalogReadings():
      stamp(),
      values_length(0), values(NULL)
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
      *(outbuffer + offset++) = values_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_valuesi;
      u_valuesi.real = this->values[i];
      *(outbuffer + offset + 0) = (u_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_valuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->values[i]);
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
      uint8_t values_lengthT = *(inbuffer + offset++);
      if(values_lengthT > values_length)
        this->values = (float*)realloc(this->values, values_lengthT * sizeof(float));
      offset += 3;
      values_length = values_lengthT;
      for( uint8_t i = 0; i < values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_values;
      u_st_values.base = 0;
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_values = u_st_values.real;
      offset += sizeof(this->st_values);
        memcpy( &(this->values[i]), &(this->st_values), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "robotino_msgs/AnalogReadings"; };
    const char * getMD5(){ return "b18badc07ef45644f603212cc1654669"; };

  };

}
#endif