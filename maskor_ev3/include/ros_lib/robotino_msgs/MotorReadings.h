#ifndef _ROS_robotino_msgs_MotorReadings_h
#define _ROS_robotino_msgs_MotorReadings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace robotino_msgs
{

  class MotorReadings : public ros::Msg
  {
    public:
      ros::Time stamp;
      uint8_t velocities_length;
      float st_velocities;
      float * velocities;
      uint8_t positions_length;
      int32_t st_positions;
      int32_t * positions;
      uint8_t currents_length;
      float st_currents;
      float * currents;

    MotorReadings():
      stamp(),
      velocities_length(0), velocities(NULL),
      positions_length(0), positions(NULL),
      currents_length(0), currents(NULL)
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
      *(outbuffer + offset++) = positions_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < positions_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_positionsi;
      u_positionsi.real = this->positions[i];
      *(outbuffer + offset + 0) = (u_positionsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positionsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positionsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positionsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->positions[i]);
      }
      *(outbuffer + offset++) = currents_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < currents_length; i++){
      union {
        float real;
        uint32_t base;
      } u_currentsi;
      u_currentsi.real = this->currents[i];
      *(outbuffer + offset + 0) = (u_currentsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_currentsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_currentsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_currentsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->currents[i]);
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
      uint8_t positions_lengthT = *(inbuffer + offset++);
      if(positions_lengthT > positions_length)
        this->positions = (int32_t*)realloc(this->positions, positions_lengthT * sizeof(int32_t));
      offset += 3;
      positions_length = positions_lengthT;
      for( uint8_t i = 0; i < positions_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_positions;
      u_st_positions.base = 0;
      u_st_positions.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_positions.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_positions.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_positions.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_positions = u_st_positions.real;
      offset += sizeof(this->st_positions);
        memcpy( &(this->positions[i]), &(this->st_positions), sizeof(int32_t));
      }
      uint8_t currents_lengthT = *(inbuffer + offset++);
      if(currents_lengthT > currents_length)
        this->currents = (float*)realloc(this->currents, currents_lengthT * sizeof(float));
      offset += 3;
      currents_length = currents_lengthT;
      for( uint8_t i = 0; i < currents_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_currents;
      u_st_currents.base = 0;
      u_st_currents.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_currents.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_currents.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_currents.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_currents = u_st_currents.real;
      offset += sizeof(this->st_currents);
        memcpy( &(this->currents[i]), &(this->st_currents), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "robotino_msgs/MotorReadings"; };
    const char * getMD5(){ return "3974e9bd8305667fc0637697b49a8e1f"; };

  };

}
#endif