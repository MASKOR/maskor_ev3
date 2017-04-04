#ifndef _ROS_robotino_msgs_EncoderReadings_h
#define _ROS_robotino_msgs_EncoderReadings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace robotino_msgs
{

  class EncoderReadings : public ros::Msg
  {
    public:
      ros::Time stamp;
      uint32_t velocity;
      uint32_t position;
      uint32_t current;

    EncoderReadings():
      stamp(),
      velocity(0),
      position(0),
      current(0)
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
      *(outbuffer + offset + 0) = (this->velocity >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity);
      *(outbuffer + offset + 0) = (this->position >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position);
      *(outbuffer + offset + 0) = (this->current >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->current >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->current >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->current >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current);
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
      this->velocity =  ((uint32_t) (*(inbuffer + offset)));
      this->velocity |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->velocity |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->velocity |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->velocity);
      this->position =  ((uint32_t) (*(inbuffer + offset)));
      this->position |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->position |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->position |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->position);
      this->current =  ((uint32_t) (*(inbuffer + offset)));
      this->current |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->current |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->current);
     return offset;
    }

    const char * getType(){ return "robotino_msgs/EncoderReadings"; };
    const char * getMD5(){ return "0b4033dda61bb04d3e2ea6c671f26183"; };

  };

}
#endif