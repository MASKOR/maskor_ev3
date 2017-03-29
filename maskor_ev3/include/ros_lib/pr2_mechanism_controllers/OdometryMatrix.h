#ifndef _ROS_pr2_mechanism_controllers_OdometryMatrix_h
#define _ROS_pr2_mechanism_controllers_OdometryMatrix_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pr2_mechanism_controllers
{

  class OdometryMatrix : public ros::Msg
  {
    public:
      uint32_t m_length;
      typedef double _m_type;
      _m_type st_m;
      _m_type * m;

    OdometryMatrix():
      m_length(0), m(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->m_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->m_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->m_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->m_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->m_length);
      for( uint32_t i = 0; i < m_length; i++){
      union {
        double real;
        uint64_t base;
      } u_mi;
      u_mi.real = this->m[i];
      *(outbuffer + offset + 0) = (u_mi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_mi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_mi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_mi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_mi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->m[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t m_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      m_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      m_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      m_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->m_length);
      if(m_lengthT > m_length)
        this->m = (double*)realloc(this->m, m_lengthT * sizeof(double));
      m_length = m_lengthT;
      for( uint32_t i = 0; i < m_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_m;
      u_st_m.base = 0;
      u_st_m.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_m.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_m.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_m.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_m.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_m.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_m.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_m.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_m = u_st_m.real;
      offset += sizeof(this->st_m);
        memcpy( &(this->m[i]), &(this->st_m), sizeof(double));
      }
     return offset;
    }

    const char * getType(){ return "pr2_mechanism_controllers/OdometryMatrix"; };
    const char * getMD5(){ return "1f7818e7ce16454badf1bee936b0ff16"; };

  };

}
#endif