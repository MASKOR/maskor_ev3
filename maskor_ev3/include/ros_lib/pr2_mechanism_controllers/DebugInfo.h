#ifndef _ROS_pr2_mechanism_controllers_DebugInfo_h
#define _ROS_pr2_mechanism_controllers_DebugInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_controllers
{

  class DebugInfo : public ros::Msg
  {
    public:
      uint32_t timing_length;
      typedef double _timing_type;
      _timing_type st_timing;
      _timing_type * timing;
      typedef int32_t _sequence_type;
      _sequence_type sequence;
      typedef bool _input_valid_type;
      _input_valid_type input_valid;
      typedef double _residual_type;
      _residual_type residual;

    DebugInfo():
      timing_length(0), timing(NULL),
      sequence(0),
      input_valid(0),
      residual(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->timing_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timing_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timing_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timing_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timing_length);
      for( uint32_t i = 0; i < timing_length; i++){
      union {
        double real;
        uint64_t base;
      } u_timingi;
      u_timingi.real = this->timing[i];
      *(outbuffer + offset + 0) = (u_timingi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timingi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_timingi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_timingi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_timingi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_timingi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_timingi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_timingi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->timing[i]);
      }
      union {
        int32_t real;
        uint32_t base;
      } u_sequence;
      u_sequence.real = this->sequence;
      *(outbuffer + offset + 0) = (u_sequence.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sequence.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sequence.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sequence.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sequence);
      union {
        bool real;
        uint8_t base;
      } u_input_valid;
      u_input_valid.real = this->input_valid;
      *(outbuffer + offset + 0) = (u_input_valid.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->input_valid);
      union {
        double real;
        uint64_t base;
      } u_residual;
      u_residual.real = this->residual;
      *(outbuffer + offset + 0) = (u_residual.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_residual.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_residual.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_residual.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_residual.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_residual.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_residual.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_residual.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->residual);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t timing_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      timing_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      timing_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      timing_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->timing_length);
      if(timing_lengthT > timing_length)
        this->timing = (double*)realloc(this->timing, timing_lengthT * sizeof(double));
      timing_length = timing_lengthT;
      for( uint32_t i = 0; i < timing_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_timing;
      u_st_timing.base = 0;
      u_st_timing.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_timing.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_timing.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_timing.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_timing.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_timing.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_timing.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_timing.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_timing = u_st_timing.real;
      offset += sizeof(this->st_timing);
        memcpy( &(this->timing[i]), &(this->st_timing), sizeof(double));
      }
      union {
        int32_t real;
        uint32_t base;
      } u_sequence;
      u_sequence.base = 0;
      u_sequence.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sequence.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sequence.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sequence.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->sequence = u_sequence.real;
      offset += sizeof(this->sequence);
      union {
        bool real;
        uint8_t base;
      } u_input_valid;
      u_input_valid.base = 0;
      u_input_valid.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->input_valid = u_input_valid.real;
      offset += sizeof(this->input_valid);
      union {
        double real;
        uint64_t base;
      } u_residual;
      u_residual.base = 0;
      u_residual.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_residual.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_residual.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_residual.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_residual.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_residual.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_residual.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_residual.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->residual = u_residual.real;
      offset += sizeof(this->residual);
     return offset;
    }

    const char * getType(){ return "pr2_mechanism_controllers/DebugInfo"; };
    const char * getMD5(){ return "6281356ce897e33da024b8eb319460f2"; };

  };

}
#endif