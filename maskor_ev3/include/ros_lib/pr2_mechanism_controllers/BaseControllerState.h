#ifndef _ROS_pr2_mechanism_controllers_BaseControllerState_h
#define _ROS_pr2_mechanism_controllers_BaseControllerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Twist.h"

namespace pr2_mechanism_controllers
{

  class BaseControllerState : public ros::Msg
  {
    public:
      typedef geometry_msgs::Twist _command_type;
      _command_type command;
      uint32_t joint_velocity_measured_length;
      typedef double _joint_velocity_measured_type;
      _joint_velocity_measured_type st_joint_velocity_measured;
      _joint_velocity_measured_type * joint_velocity_measured;
      uint32_t joint_velocity_commanded_length;
      typedef double _joint_velocity_commanded_type;
      _joint_velocity_commanded_type st_joint_velocity_commanded;
      _joint_velocity_commanded_type * joint_velocity_commanded;
      uint32_t joint_velocity_error_length;
      typedef double _joint_velocity_error_type;
      _joint_velocity_error_type st_joint_velocity_error;
      _joint_velocity_error_type * joint_velocity_error;
      uint32_t joint_effort_measured_length;
      typedef double _joint_effort_measured_type;
      _joint_effort_measured_type st_joint_effort_measured;
      _joint_effort_measured_type * joint_effort_measured;
      uint32_t joint_effort_commanded_length;
      typedef double _joint_effort_commanded_type;
      _joint_effort_commanded_type st_joint_effort_commanded;
      _joint_effort_commanded_type * joint_effort_commanded;
      uint32_t joint_effort_error_length;
      typedef double _joint_effort_error_type;
      _joint_effort_error_type st_joint_effort_error;
      _joint_effort_error_type * joint_effort_error;
      uint32_t joint_names_length;
      typedef char* _joint_names_type;
      _joint_names_type st_joint_names;
      _joint_names_type * joint_names;

    BaseControllerState():
      command(),
      joint_velocity_measured_length(0), joint_velocity_measured(NULL),
      joint_velocity_commanded_length(0), joint_velocity_commanded(NULL),
      joint_velocity_error_length(0), joint_velocity_error(NULL),
      joint_effort_measured_length(0), joint_effort_measured(NULL),
      joint_effort_commanded_length(0), joint_effort_commanded(NULL),
      joint_effort_error_length(0), joint_effort_error(NULL),
      joint_names_length(0), joint_names(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->command.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->joint_velocity_measured_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_velocity_measured_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_velocity_measured_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_velocity_measured_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_velocity_measured_length);
      for( uint32_t i = 0; i < joint_velocity_measured_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_velocity_measuredi;
      u_joint_velocity_measuredi.real = this->joint_velocity_measured[i];
      *(outbuffer + offset + 0) = (u_joint_velocity_measuredi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_velocity_measuredi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_velocity_measuredi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_velocity_measuredi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_velocity_measuredi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_velocity_measuredi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_velocity_measuredi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_velocity_measuredi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_velocity_measured[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_velocity_commanded_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_velocity_commanded_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_velocity_commanded_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_velocity_commanded_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_velocity_commanded_length);
      for( uint32_t i = 0; i < joint_velocity_commanded_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_velocity_commandedi;
      u_joint_velocity_commandedi.real = this->joint_velocity_commanded[i];
      *(outbuffer + offset + 0) = (u_joint_velocity_commandedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_velocity_commandedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_velocity_commandedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_velocity_commandedi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_velocity_commandedi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_velocity_commandedi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_velocity_commandedi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_velocity_commandedi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_velocity_commanded[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_velocity_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_velocity_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_velocity_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_velocity_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_velocity_error_length);
      for( uint32_t i = 0; i < joint_velocity_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_velocity_errori;
      u_joint_velocity_errori.real = this->joint_velocity_error[i];
      *(outbuffer + offset + 0) = (u_joint_velocity_errori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_velocity_errori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_velocity_errori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_velocity_errori.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_velocity_errori.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_velocity_errori.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_velocity_errori.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_velocity_errori.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_velocity_error[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_effort_measured_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_effort_measured_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_effort_measured_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_effort_measured_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_effort_measured_length);
      for( uint32_t i = 0; i < joint_effort_measured_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_effort_measuredi;
      u_joint_effort_measuredi.real = this->joint_effort_measured[i];
      *(outbuffer + offset + 0) = (u_joint_effort_measuredi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_effort_measuredi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_effort_measuredi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_effort_measuredi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_effort_measuredi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_effort_measuredi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_effort_measuredi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_effort_measuredi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_effort_measured[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_effort_commanded_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_effort_commanded_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_effort_commanded_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_effort_commanded_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_effort_commanded_length);
      for( uint32_t i = 0; i < joint_effort_commanded_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_effort_commandedi;
      u_joint_effort_commandedi.real = this->joint_effort_commanded[i];
      *(outbuffer + offset + 0) = (u_joint_effort_commandedi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_effort_commandedi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_effort_commandedi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_effort_commandedi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_effort_commandedi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_effort_commandedi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_effort_commandedi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_effort_commandedi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_effort_commanded[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_effort_error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_effort_error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_effort_error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_effort_error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_effort_error_length);
      for( uint32_t i = 0; i < joint_effort_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_effort_errori;
      u_joint_effort_errori.real = this->joint_effort_error[i];
      *(outbuffer + offset + 0) = (u_joint_effort_errori.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_effort_errori.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_effort_errori.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_effort_errori.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_effort_errori.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_effort_errori.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_effort_errori.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_effort_errori.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_effort_error[i]);
      }
      *(outbuffer + offset + 0) = (this->joint_names_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_names_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_names_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_names_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_names_length);
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_joint_namesi = strlen(this->joint_names[i]);
      varToArr(outbuffer + offset, length_joint_namesi);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_names[i], length_joint_namesi);
      offset += length_joint_namesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->command.deserialize(inbuffer + offset);
      uint32_t joint_velocity_measured_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_velocity_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_velocity_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_velocity_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_velocity_measured_length);
      if(joint_velocity_measured_lengthT > joint_velocity_measured_length)
        this->joint_velocity_measured = (double*)realloc(this->joint_velocity_measured, joint_velocity_measured_lengthT * sizeof(double));
      joint_velocity_measured_length = joint_velocity_measured_lengthT;
      for( uint32_t i = 0; i < joint_velocity_measured_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_velocity_measured;
      u_st_joint_velocity_measured.base = 0;
      u_st_joint_velocity_measured.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_velocity_measured.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_velocity_measured.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_velocity_measured.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_velocity_measured.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_velocity_measured.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_velocity_measured.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_velocity_measured.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_velocity_measured = u_st_joint_velocity_measured.real;
      offset += sizeof(this->st_joint_velocity_measured);
        memcpy( &(this->joint_velocity_measured[i]), &(this->st_joint_velocity_measured), sizeof(double));
      }
      uint32_t joint_velocity_commanded_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_velocity_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_velocity_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_velocity_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_velocity_commanded_length);
      if(joint_velocity_commanded_lengthT > joint_velocity_commanded_length)
        this->joint_velocity_commanded = (double*)realloc(this->joint_velocity_commanded, joint_velocity_commanded_lengthT * sizeof(double));
      joint_velocity_commanded_length = joint_velocity_commanded_lengthT;
      for( uint32_t i = 0; i < joint_velocity_commanded_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_velocity_commanded;
      u_st_joint_velocity_commanded.base = 0;
      u_st_joint_velocity_commanded.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_velocity_commanded.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_velocity_commanded.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_velocity_commanded.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_velocity_commanded.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_velocity_commanded.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_velocity_commanded.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_velocity_commanded.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_velocity_commanded = u_st_joint_velocity_commanded.real;
      offset += sizeof(this->st_joint_velocity_commanded);
        memcpy( &(this->joint_velocity_commanded[i]), &(this->st_joint_velocity_commanded), sizeof(double));
      }
      uint32_t joint_velocity_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_velocity_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_velocity_error_length);
      if(joint_velocity_error_lengthT > joint_velocity_error_length)
        this->joint_velocity_error = (double*)realloc(this->joint_velocity_error, joint_velocity_error_lengthT * sizeof(double));
      joint_velocity_error_length = joint_velocity_error_lengthT;
      for( uint32_t i = 0; i < joint_velocity_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_velocity_error;
      u_st_joint_velocity_error.base = 0;
      u_st_joint_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_velocity_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_velocity_error = u_st_joint_velocity_error.real;
      offset += sizeof(this->st_joint_velocity_error);
        memcpy( &(this->joint_velocity_error[i]), &(this->st_joint_velocity_error), sizeof(double));
      }
      uint32_t joint_effort_measured_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_effort_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_effort_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_effort_measured_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_effort_measured_length);
      if(joint_effort_measured_lengthT > joint_effort_measured_length)
        this->joint_effort_measured = (double*)realloc(this->joint_effort_measured, joint_effort_measured_lengthT * sizeof(double));
      joint_effort_measured_length = joint_effort_measured_lengthT;
      for( uint32_t i = 0; i < joint_effort_measured_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_effort_measured;
      u_st_joint_effort_measured.base = 0;
      u_st_joint_effort_measured.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_effort_measured.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_effort_measured.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_effort_measured.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_effort_measured.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_effort_measured.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_effort_measured.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_effort_measured.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_effort_measured = u_st_joint_effort_measured.real;
      offset += sizeof(this->st_joint_effort_measured);
        memcpy( &(this->joint_effort_measured[i]), &(this->st_joint_effort_measured), sizeof(double));
      }
      uint32_t joint_effort_commanded_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_effort_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_effort_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_effort_commanded_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_effort_commanded_length);
      if(joint_effort_commanded_lengthT > joint_effort_commanded_length)
        this->joint_effort_commanded = (double*)realloc(this->joint_effort_commanded, joint_effort_commanded_lengthT * sizeof(double));
      joint_effort_commanded_length = joint_effort_commanded_lengthT;
      for( uint32_t i = 0; i < joint_effort_commanded_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_effort_commanded;
      u_st_joint_effort_commanded.base = 0;
      u_st_joint_effort_commanded.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_effort_commanded.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_effort_commanded.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_effort_commanded.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_effort_commanded.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_effort_commanded.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_effort_commanded.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_effort_commanded.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_effort_commanded = u_st_joint_effort_commanded.real;
      offset += sizeof(this->st_joint_effort_commanded);
        memcpy( &(this->joint_effort_commanded[i]), &(this->st_joint_effort_commanded), sizeof(double));
      }
      uint32_t joint_effort_error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_effort_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_effort_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_effort_error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_effort_error_length);
      if(joint_effort_error_lengthT > joint_effort_error_length)
        this->joint_effort_error = (double*)realloc(this->joint_effort_error, joint_effort_error_lengthT * sizeof(double));
      joint_effort_error_length = joint_effort_error_lengthT;
      for( uint32_t i = 0; i < joint_effort_error_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_joint_effort_error;
      u_st_joint_effort_error.base = 0;
      u_st_joint_effort_error.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_joint_effort_error.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_joint_effort_error.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_joint_effort_error.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_joint_effort_error.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_joint_effort_error.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_joint_effort_error.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_joint_effort_error.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_joint_effort_error = u_st_joint_effort_error.real;
      offset += sizeof(this->st_joint_effort_error);
        memcpy( &(this->joint_effort_error[i]), &(this->st_joint_effort_error), sizeof(double));
      }
      uint32_t joint_names_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_names_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_names_length);
      if(joint_names_lengthT > joint_names_length)
        this->joint_names = (char**)realloc(this->joint_names, joint_names_lengthT * sizeof(char*));
      joint_names_length = joint_names_lengthT;
      for( uint32_t i = 0; i < joint_names_length; i++){
      uint32_t length_st_joint_names;
      arrToVar(length_st_joint_names, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_names; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_names-1]=0;
      this->st_joint_names = (char *)(inbuffer + offset-1);
      offset += length_st_joint_names;
        memcpy( &(this->joint_names[i]), &(this->st_joint_names), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "pr2_mechanism_controllers/BaseControllerState"; };
    const char * getMD5(){ return "7a488aa492f9175d5fa35e22e56c4b28"; };

  };

}
#endif