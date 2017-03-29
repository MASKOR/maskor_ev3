#ifndef _ROS_SERVICE_SetProfile_h
#define _ROS_SERVICE_SetProfile_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_controllers
{

static const char SETPROFILE[] = "pr2_mechanism_controllers/SetProfile";

  class SetProfileRequest : public ros::Msg
  {
    public:
      typedef double _UpperTurnaround_type;
      _UpperTurnaround_type UpperTurnaround;
      typedef double _LowerTurnaround_type;
      _LowerTurnaround_type LowerTurnaround;
      typedef double _upperDecelBuffer_type;
      _upperDecelBuffer_type upperDecelBuffer;
      typedef double _lowerDecelBuffer_type;
      _lowerDecelBuffer_type lowerDecelBuffer;
      typedef double _profile_type;
      _profile_type profile;
      typedef double _period_type;
      _period_type period;
      typedef double _amplitude_type;
      _amplitude_type amplitude;
      typedef double _offset_type;
      _offset_type offset;

    SetProfileRequest():
      UpperTurnaround(0),
      LowerTurnaround(0),
      upperDecelBuffer(0),
      lowerDecelBuffer(0),
      profile(0),
      period(0),
      amplitude(0),
      offset(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_UpperTurnaround;
      u_UpperTurnaround.real = this->UpperTurnaround;
      *(outbuffer + offset + 0) = (u_UpperTurnaround.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_UpperTurnaround.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_UpperTurnaround.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_UpperTurnaround.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_UpperTurnaround.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_UpperTurnaround.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_UpperTurnaround.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_UpperTurnaround.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->UpperTurnaround);
      union {
        double real;
        uint64_t base;
      } u_LowerTurnaround;
      u_LowerTurnaround.real = this->LowerTurnaround;
      *(outbuffer + offset + 0) = (u_LowerTurnaround.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_LowerTurnaround.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_LowerTurnaround.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_LowerTurnaround.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_LowerTurnaround.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_LowerTurnaround.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_LowerTurnaround.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_LowerTurnaround.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->LowerTurnaround);
      union {
        double real;
        uint64_t base;
      } u_upperDecelBuffer;
      u_upperDecelBuffer.real = this->upperDecelBuffer;
      *(outbuffer + offset + 0) = (u_upperDecelBuffer.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_upperDecelBuffer.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_upperDecelBuffer.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_upperDecelBuffer.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_upperDecelBuffer.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_upperDecelBuffer.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_upperDecelBuffer.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_upperDecelBuffer.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->upperDecelBuffer);
      union {
        double real;
        uint64_t base;
      } u_lowerDecelBuffer;
      u_lowerDecelBuffer.real = this->lowerDecelBuffer;
      *(outbuffer + offset + 0) = (u_lowerDecelBuffer.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_lowerDecelBuffer.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_lowerDecelBuffer.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_lowerDecelBuffer.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_lowerDecelBuffer.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_lowerDecelBuffer.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_lowerDecelBuffer.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_lowerDecelBuffer.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->lowerDecelBuffer);
      union {
        double real;
        uint64_t base;
      } u_profile;
      u_profile.real = this->profile;
      *(outbuffer + offset + 0) = (u_profile.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_profile.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_profile.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_profile.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_profile.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_profile.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_profile.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_profile.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->profile);
      union {
        double real;
        uint64_t base;
      } u_period;
      u_period.real = this->period;
      *(outbuffer + offset + 0) = (u_period.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_period.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_period.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_period.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_period.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_period.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_period.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_period.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->period);
      union {
        double real;
        uint64_t base;
      } u_amplitude;
      u_amplitude.real = this->amplitude;
      *(outbuffer + offset + 0) = (u_amplitude.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_amplitude.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_amplitude.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_amplitude.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_amplitude.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_amplitude.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_amplitude.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_amplitude.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->amplitude);
      union {
        double real;
        uint64_t base;
      } u_offset;
      u_offset.real = this->offset;
      *(outbuffer + offset + 0) = (u_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_offset.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_offset.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_offset.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_offset.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_offset.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_UpperTurnaround;
      u_UpperTurnaround.base = 0;
      u_UpperTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_UpperTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_UpperTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_UpperTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_UpperTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_UpperTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_UpperTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_UpperTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->UpperTurnaround = u_UpperTurnaround.real;
      offset += sizeof(this->UpperTurnaround);
      union {
        double real;
        uint64_t base;
      } u_LowerTurnaround;
      u_LowerTurnaround.base = 0;
      u_LowerTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_LowerTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_LowerTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_LowerTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_LowerTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_LowerTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_LowerTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_LowerTurnaround.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->LowerTurnaround = u_LowerTurnaround.real;
      offset += sizeof(this->LowerTurnaround);
      union {
        double real;
        uint64_t base;
      } u_upperDecelBuffer;
      u_upperDecelBuffer.base = 0;
      u_upperDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_upperDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_upperDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_upperDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_upperDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_upperDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_upperDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_upperDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->upperDecelBuffer = u_upperDecelBuffer.real;
      offset += sizeof(this->upperDecelBuffer);
      union {
        double real;
        uint64_t base;
      } u_lowerDecelBuffer;
      u_lowerDecelBuffer.base = 0;
      u_lowerDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_lowerDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_lowerDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_lowerDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_lowerDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_lowerDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_lowerDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_lowerDecelBuffer.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->lowerDecelBuffer = u_lowerDecelBuffer.real;
      offset += sizeof(this->lowerDecelBuffer);
      union {
        double real;
        uint64_t base;
      } u_profile;
      u_profile.base = 0;
      u_profile.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_profile.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_profile.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_profile.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_profile.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_profile.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_profile.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_profile.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->profile = u_profile.real;
      offset += sizeof(this->profile);
      union {
        double real;
        uint64_t base;
      } u_period;
      u_period.base = 0;
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_period.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->period = u_period.real;
      offset += sizeof(this->period);
      union {
        double real;
        uint64_t base;
      } u_amplitude;
      u_amplitude.base = 0;
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_amplitude.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->amplitude = u_amplitude.real;
      offset += sizeof(this->amplitude);
      union {
        double real;
        uint64_t base;
      } u_offset;
      u_offset.base = 0;
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_offset.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->offset = u_offset.real;
      offset += sizeof(this->offset);
     return offset;
    }

    const char * getType(){ return SETPROFILE; };
    const char * getMD5(){ return "309001fc196b0094f23b1ae2bd672fb2"; };

  };

  class SetProfileResponse : public ros::Msg
  {
    public:
      typedef double _time_type;
      _time_type time;

    SetProfileResponse():
      time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.real = this->time;
      *(outbuffer + offset + 0) = (u_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_time;
      u_time.base = 0;
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->time = u_time.real;
      offset += sizeof(this->time);
     return offset;
    }

    const char * getType(){ return SETPROFILE; };
    const char * getMD5(){ return "be5310e7aa4c90cdee120add91648cee"; };

  };

  class SetProfile {
    public:
    typedef SetProfileRequest Request;
    typedef SetProfileResponse Response;
  };

}
#endif
