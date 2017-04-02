#ifndef _ROS_maskor_ev3_msgs_InfraredSensor_h
#define _ROS_maskor_ev3_msgs_InfraredSensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace maskor_ev3_msgs
{

  class InfraredSensor : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t proximity;

    InfraredSensor():
      header(),
      proximity(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->proximity >> (8 * 0)) & 0xFF;
      offset += sizeof(this->proximity);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->proximity =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->proximity);
     return offset;
    }

    const char * getType(){ return "maskor_ev3_msgs/InfraredSensor"; };
    const char * getMD5(){ return "26d91c28008e7aed6e60d0e05a02f32c"; };

  };

}
#endif