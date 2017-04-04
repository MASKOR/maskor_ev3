#ifndef _ROS_robotino_msgs_GrapplerReadings_h
#define _ROS_robotino_msgs_GrapplerReadings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace robotino_msgs
{

  class GrapplerReadings : public ros::Msg
  {
    public:
      ros::Time stamp;
      uint32_t seq;
      uint32_t numServos;
      bool torqueEnabled;
      uint8_t angles_length;
      float st_angles;
      float * angles;
      uint8_t velocities_length;
      float st_velocities;
      float * velocities;
      uint8_t errors_length;
      uint32_t st_errors;
      uint32_t * errors;
      uint8_t channels_length;
      uint32_t st_channels;
      uint32_t * channels;
      uint8_t ids_length;
      uint32_t st_ids;
      uint32_t * ids;
      uint8_t cwAxesLimits_length;
      float st_cwAxesLimits;
      float * cwAxesLimits;
      uint8_t ccwAxesLimits_length;
      float st_ccwAxesLimits;
      float * ccwAxesLimits;

    GrapplerReadings():
      stamp(),
      seq(0),
      numServos(0),
      torqueEnabled(0),
      angles_length(0), angles(NULL),
      velocities_length(0), velocities(NULL),
      errors_length(0), errors(NULL),
      channels_length(0), channels(NULL),
      ids_length(0), ids(NULL),
      cwAxesLimits_length(0), cwAxesLimits(NULL),
      ccwAxesLimits_length(0), ccwAxesLimits(NULL)
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
      *(outbuffer + offset + 0) = (this->numServos >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->numServos >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->numServos >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->numServos >> (8 * 3)) & 0xFF;
      offset += sizeof(this->numServos);
      union {
        bool real;
        uint8_t base;
      } u_torqueEnabled;
      u_torqueEnabled.real = this->torqueEnabled;
      *(outbuffer + offset + 0) = (u_torqueEnabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->torqueEnabled);
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
      *(outbuffer + offset++) = errors_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < errors_length; i++){
      *(outbuffer + offset + 0) = (this->errors[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->errors[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->errors[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->errors[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->errors[i]);
      }
      *(outbuffer + offset++) = channels_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < channels_length; i++){
      *(outbuffer + offset + 0) = (this->channels[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->channels[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->channels[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->channels[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->channels[i]);
      }
      *(outbuffer + offset++) = ids_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < ids_length; i++){
      *(outbuffer + offset + 0) = (this->ids[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->ids[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->ids[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->ids[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ids[i]);
      }
      *(outbuffer + offset++) = cwAxesLimits_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cwAxesLimits_length; i++){
      union {
        float real;
        uint32_t base;
      } u_cwAxesLimitsi;
      u_cwAxesLimitsi.real = this->cwAxesLimits[i];
      *(outbuffer + offset + 0) = (u_cwAxesLimitsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cwAxesLimitsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cwAxesLimitsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cwAxesLimitsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cwAxesLimits[i]);
      }
      *(outbuffer + offset++) = ccwAxesLimits_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < ccwAxesLimits_length; i++){
      union {
        float real;
        uint32_t base;
      } u_ccwAxesLimitsi;
      u_ccwAxesLimitsi.real = this->ccwAxesLimits[i];
      *(outbuffer + offset + 0) = (u_ccwAxesLimitsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_ccwAxesLimitsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_ccwAxesLimitsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_ccwAxesLimitsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->ccwAxesLimits[i]);
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
      this->seq =  ((uint32_t) (*(inbuffer + offset)));
      this->seq |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->seq |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->seq |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->seq);
      this->numServos =  ((uint32_t) (*(inbuffer + offset)));
      this->numServos |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->numServos |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->numServos |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->numServos);
      union {
        bool real;
        uint8_t base;
      } u_torqueEnabled;
      u_torqueEnabled.base = 0;
      u_torqueEnabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->torqueEnabled = u_torqueEnabled.real;
      offset += sizeof(this->torqueEnabled);
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
      uint8_t errors_lengthT = *(inbuffer + offset++);
      if(errors_lengthT > errors_length)
        this->errors = (uint32_t*)realloc(this->errors, errors_lengthT * sizeof(uint32_t));
      offset += 3;
      errors_length = errors_lengthT;
      for( uint8_t i = 0; i < errors_length; i++){
      this->st_errors =  ((uint32_t) (*(inbuffer + offset)));
      this->st_errors |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_errors |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_errors |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_errors);
        memcpy( &(this->errors[i]), &(this->st_errors), sizeof(uint32_t));
      }
      uint8_t channels_lengthT = *(inbuffer + offset++);
      if(channels_lengthT > channels_length)
        this->channels = (uint32_t*)realloc(this->channels, channels_lengthT * sizeof(uint32_t));
      offset += 3;
      channels_length = channels_lengthT;
      for( uint8_t i = 0; i < channels_length; i++){
      this->st_channels =  ((uint32_t) (*(inbuffer + offset)));
      this->st_channels |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_channels |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_channels |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_channels);
        memcpy( &(this->channels[i]), &(this->st_channels), sizeof(uint32_t));
      }
      uint8_t ids_lengthT = *(inbuffer + offset++);
      if(ids_lengthT > ids_length)
        this->ids = (uint32_t*)realloc(this->ids, ids_lengthT * sizeof(uint32_t));
      offset += 3;
      ids_length = ids_lengthT;
      for( uint8_t i = 0; i < ids_length; i++){
      this->st_ids =  ((uint32_t) (*(inbuffer + offset)));
      this->st_ids |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_ids |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_ids |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_ids);
        memcpy( &(this->ids[i]), &(this->st_ids), sizeof(uint32_t));
      }
      uint8_t cwAxesLimits_lengthT = *(inbuffer + offset++);
      if(cwAxesLimits_lengthT > cwAxesLimits_length)
        this->cwAxesLimits = (float*)realloc(this->cwAxesLimits, cwAxesLimits_lengthT * sizeof(float));
      offset += 3;
      cwAxesLimits_length = cwAxesLimits_lengthT;
      for( uint8_t i = 0; i < cwAxesLimits_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_cwAxesLimits;
      u_st_cwAxesLimits.base = 0;
      u_st_cwAxesLimits.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_cwAxesLimits.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_cwAxesLimits.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_cwAxesLimits.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_cwAxesLimits = u_st_cwAxesLimits.real;
      offset += sizeof(this->st_cwAxesLimits);
        memcpy( &(this->cwAxesLimits[i]), &(this->st_cwAxesLimits), sizeof(float));
      }
      uint8_t ccwAxesLimits_lengthT = *(inbuffer + offset++);
      if(ccwAxesLimits_lengthT > ccwAxesLimits_length)
        this->ccwAxesLimits = (float*)realloc(this->ccwAxesLimits, ccwAxesLimits_lengthT * sizeof(float));
      offset += 3;
      ccwAxesLimits_length = ccwAxesLimits_lengthT;
      for( uint8_t i = 0; i < ccwAxesLimits_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_ccwAxesLimits;
      u_st_ccwAxesLimits.base = 0;
      u_st_ccwAxesLimits.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_ccwAxesLimits.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_ccwAxesLimits.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_ccwAxesLimits.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_ccwAxesLimits = u_st_ccwAxesLimits.real;
      offset += sizeof(this->st_ccwAxesLimits);
        memcpy( &(this->ccwAxesLimits[i]), &(this->st_ccwAxesLimits), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "robotino_msgs/GrapplerReadings"; };
    const char * getMD5(){ return "53d1f6c81df9b5242320201fe0231738"; };

  };

}
#endif