#ifndef _ROS_SERVICE_SetEncoderPosition_h
#define _ROS_SERVICE_SetEncoderPosition_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace robotino_msgs
{

static const char SETENCODERPOSITION[] = "robotino_msgs/SetEncoderPosition";

  class SetEncoderPositionRequest : public ros::Msg
  {
    public:
      uint32_t position;
      uint32_t velocity;

    SetEncoderPositionRequest():
      position(0),
      velocity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->position >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position);
      *(outbuffer + offset + 0) = (this->velocity >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->position =  ((uint32_t) (*(inbuffer + offset)));
      this->position |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->position |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->position);
      this->velocity =  ((uint32_t) (*(inbuffer + offset)));
      this->velocity |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->velocity |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->velocity |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->velocity);
     return offset;
    }

    const char * getType(){ return SETENCODERPOSITION; };
    const char * getMD5(){ return "e17f4b08b3e2cdbbf2b0d78e4f62e5d6"; };

  };

  class SetEncoderPositionResponse : public ros::Msg
  {
    public:

    SetEncoderPositionResponse()
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

    const char * getType(){ return SETENCODERPOSITION; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetEncoderPosition {
    public:
    typedef SetEncoderPositionRequest Request;
    typedef SetEncoderPositionResponse Response;
  };

}
#endif
