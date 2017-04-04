#ifndef _ROS_robotino_msgs_SetGrapplerAxes_h
#define _ROS_robotino_msgs_SetGrapplerAxes_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace robotino_msgs
{

  class SetGrapplerAxes : public ros::Msg
  {
    public:
      ros::Time stamp;
      uint8_t angles_length;
      float st_angles;
      float * angles;
      uint8_t velocities_length;
      float st_velocities;
      float * velocities;

    SetGrapplerAxes():
      stamp(),
      angles_length(0), angles(NULL),
      velocities_length(0), velocities(NULL)
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
      *(outbuffer + offset++) = angles_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < angles_length; i++){
      union {
        float real;
        uint32_t base;
      } u_anglesi;
      u_anglesi.real = this->angles[i];
      *(outbuffer + offset + 0) = (u_anglesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_anglesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_anglesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_anglesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angles[i]);
      }
      *(outbuffer + offset++) = velocities_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < velocities_length; i++){
      union {
        float real;
        uint32_t base;
      } u_velocitiesi;
      u_velocitiesi.real = this->velocities[i];
      *(outbuffer + offset + 0) = (u_velocitiesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_velocitiesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_velocitiesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_velocitiesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocities[i]);
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
      uint8_t angles_lengthT = *(inbuffer + offset++);
      if(angles_lengthT > angles_length)
        this->angles = (float*)realloc(this->angles, angles_lengthT * sizeof(float));
      offset += 3;
      angles_length = angles_lengthT;
      for( uint8_t i = 0; i < angles_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_angles;
      u_st_angles.base = 0;
      u_st_angles.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_angles.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_angles.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_angles.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_angles = u_st_angles.real;
      offset += sizeof(this->st_angles);
        memcpy( &(this->angles[i]), &(this->st_angles), sizeof(float));
      }
      uint8_t velocities_lengthT = *(inbuffer + offset++);
      if(velocities_lengthT > velocities_length)
        this->velocities = (float*)realloc(this->velocities, velocities_lengthT * sizeof(float));
      offset += 3;
      velocities_length = velocities_lengthT;
      for( uint8_t i = 0; i < velocities_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_velocities;
      u_st_velocities.base = 0;
      u_st_velocities.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_velocities.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_velocities.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_velocities.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_velocities = u_st_velocities.real;
      offset += sizeof(this->st_velocities);
        memcpy( &(this->velocities[i]), &(this->st_velocities), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "robotino_msgs/SetGrapplerAxes"; };
    const char * getMD5(){ return "2d3c214086c6fdfe1e85f9e5abf4507a"; };

  };

}
#endif