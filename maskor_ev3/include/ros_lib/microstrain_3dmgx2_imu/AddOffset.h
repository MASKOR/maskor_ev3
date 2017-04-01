#ifndef _ROS_SERVICE_AddOffset_h
#define _ROS_SERVICE_AddOffset_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace microstrain_3dmgx2_imu
{

static const char ADDOFFSET[] = "microstrain_3dmgx2_imu/AddOffset";

  class AddOffsetRequest : public ros::Msg
  {
    public:
      double add_offset;

    AddOffsetRequest():
      add_offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_add_offset;
      u_add_offset.real = this->add_offset;
      *(outbuffer + offset + 0) = (u_add_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_add_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_add_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_add_offset.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_add_offset.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_add_offset.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_add_offset.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_add_offset.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->add_offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_add_offset;
      u_add_offset.base = 0;
      u_add_offset.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_add_offset.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_add_offset.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_add_offset.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_add_offset.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_add_offset.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_add_offset.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_add_offset.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->add_offset = u_add_offset.real;
      offset += sizeof(this->add_offset);
     return offset;
    }

    const char * getType(){ return ADDOFFSET; };
    const char * getMD5(){ return "10fe27c5d4591264b9d05acc7497a18a"; };

  };

  class AddOffsetResponse : public ros::Msg
  {
    public:
      double total_offset;

    AddOffsetResponse():
      total_offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_total_offset;
      u_total_offset.real = this->total_offset;
      *(outbuffer + offset + 0) = (u_total_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_total_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_total_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_total_offset.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_total_offset.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_total_offset.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_total_offset.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_total_offset.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->total_offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_total_offset;
      u_total_offset.base = 0;
      u_total_offset.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_total_offset.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_total_offset.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_total_offset.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_total_offset.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_total_offset.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_total_offset.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_total_offset.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->total_offset = u_total_offset.real;
      offset += sizeof(this->total_offset);
     return offset;
    }

    const char * getType(){ return ADDOFFSET; };
    const char * getMD5(){ return "5dea42ce4656fada4736ce3508b56aca"; };

  };

  class AddOffset {
    public:
    typedef AddOffsetRequest Request;
    typedef AddOffsetResponse Response;
  };

}
#endif
