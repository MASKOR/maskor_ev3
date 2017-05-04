#ifndef _ROS_maskor_ev3_msgs_ColorSensor_h
#define _ROS_maskor_ev3_msgs_ColorSensor_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace maskor_ev3_msgs
{

  class ColorSensor : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint8_t _color_type;
      _color_type color;

    ColorSensor():
      header(),
      color(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->color >> (8 * 0)) & 0xFF;
      offset += sizeof(this->color);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->color =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->color);
     return offset;
    }

    const char * getType(){ return "maskor_ev3_msgs/ColorSensor"; };
    const char * getMD5(){ return "36f63d241a341fc5ac60fd9ee64d4836"; };

  };

}
#endif