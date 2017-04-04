#ifndef _ROS_SERVICE_ResetOdometry_h
#define _ROS_SERVICE_ResetOdometry_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotino_msgs
{

static const char RESETODOMETRY[] = "robotino_msgs/ResetOdometry";

  class ResetOdometryRequest : public ros::Msg
  {
    public:
      float x;
      float y;
      float phi;

    ResetOdometryRequest():
      x(0),
      y(0),
      phi(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_phi;
      u_phi.real = this->phi;
      *(outbuffer + offset + 0) = (u_phi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_phi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_phi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_phi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->phi);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        uint32_t base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        uint32_t base;
      } u_phi;
      u_phi.base = 0;
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_phi.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->phi = u_phi.real;
      offset += sizeof(this->phi);
     return offset;
    }

    const char * getType(){ return RESETODOMETRY; };
    const char * getMD5(){ return "400210218ff66e5bce9e5be16653c0c8"; };

  };

  class ResetOdometryResponse : public ros::Msg
  {
    public:

    ResetOdometryResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return RESETODOMETRY; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class ResetOdometry {
    public:
    typedef ResetOdometryRequest Request;
    typedef ResetOdometryResponse Response;
  };

}
#endif
