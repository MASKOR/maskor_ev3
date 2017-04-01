#ifndef _ROS_robotino_msgs_NorthStarReadings_h
#define _ROS_robotino_msgs_NorthStarReadings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "geometry_msgs/Pose.h"

namespace robotino_msgs
{

  class NorthStarReadings : public ros::Msg
  {
    public:
      ros::Time stamp;
      uint32_t seq;
      uint32_t roomId;
      uint32_t numSpotsVisible;
      geometry_msgs::Pose pose;
      uint32_t magSpot0;
      uint32_t magSpot1;

    NorthStarReadings():
      stamp(),
      seq(0),
      roomId(0),
      numSpotsVisible(0),
      pose(),
      magSpot0(0),
      magSpot1(0)
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
      *(outbuffer + offset + 0) = (this->seq >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seq >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->seq >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->seq >> (8 * 3)) & 0xFF;
      offset += sizeof(this->seq);
      *(outbuffer + offset + 0) = (this->roomId >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->roomId >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->roomId >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->roomId >> (8 * 3)) & 0xFF;
      offset += sizeof(this->roomId);
      *(outbuffer + offset + 0) = (this->numSpotsVisible >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->numSpotsVisible >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->numSpotsVisible >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->numSpotsVisible >> (8 * 3)) & 0xFF;
      offset += sizeof(this->numSpotsVisible);
      offset += this->pose.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->magSpot0 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->magSpot0 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->magSpot0 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->magSpot0 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->magSpot0);
      *(outbuffer + offset + 0) = (this->magSpot1 >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->magSpot1 >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->magSpot1 >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->magSpot1 >> (8 * 3)) & 0xFF;
      offset += sizeof(this->magSpot1);
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
      this->seq =  ((uint32_t) (*(inbuffer + offset)));
      this->seq |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->seq |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->seq |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->seq);
      this->roomId =  ((uint32_t) (*(inbuffer + offset)));
      this->roomId |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->roomId |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->roomId |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->roomId);
      this->numSpotsVisible =  ((uint32_t) (*(inbuffer + offset)));
      this->numSpotsVisible |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->numSpotsVisible |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->numSpotsVisible |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->numSpotsVisible);
      offset += this->pose.deserialize(inbuffer + offset);
      this->magSpot0 =  ((uint32_t) (*(inbuffer + offset)));
      this->magSpot0 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->magSpot0 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->magSpot0 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->magSpot0);
      this->magSpot1 =  ((uint32_t) (*(inbuffer + offset)));
      this->magSpot1 |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->magSpot1 |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->magSpot1 |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->magSpot1);
     return offset;
    }

    const char * getType(){ return "robotino_msgs/NorthStarReadings"; };
    const char * getMD5(){ return "b8db44cc88a378f5282e8add1661d1e7"; };

  };

}
#endif