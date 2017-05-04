#ifndef _ROS_maskor_ev3_msgs_UltrasonicSensor_h
#define _ROS_maskor_ev3_msgs_UltrasonicSensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace maskor_ev3_msgs
{

  class UltrasonicSensor : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint16_t _distance_type;
      _distance_type distance;

    UltrasonicSensor():
      header(),
      distance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->distance >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->distance >> (8 * 1)) & 0xFF;
      offset += sizeof(this->distance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->distance =  ((uint16_t) (*(inbuffer + offset)));
      this->distance |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->distance);
     return offset;
    }

    const char * getType(){ return "maskor_ev3_msgs/UltrasonicSensor"; };
    const char * getMD5(){ return "e060ee2b8a21894e35cf1a8fc4fe278e"; };

  };

}
#endif