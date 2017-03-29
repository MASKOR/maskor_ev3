#ifndef _ROS_SERVICE_GetModelList_h
#define _ROS_SERVICE_GetModelList_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "household_objects_database_msgs/DatabaseReturnCode.h"

namespace household_objects_database_msgs
{

static const char GETMODELLIST[] = "household_objects_database_msgs/GetModelList";

  class GetModelListRequest : public ros::Msg
  {
    public:
      typedef const char* _model_set_type;
      _model_set_type model_set;

    GetModelListRequest():
      model_set("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_model_set = strlen(this->model_set);
      varToArr(outbuffer + offset, length_model_set);
      offset += 4;
      memcpy(outbuffer + offset, this->model_set, length_model_set);
      offset += length_model_set;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_model_set;
      arrToVar(length_model_set, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model_set; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model_set-1]=0;
      this->model_set = (char *)(inbuffer + offset-1);
      offset += length_model_set;
     return offset;
    }

    const char * getType(){ return GETMODELLIST; };
    const char * getMD5(){ return "6bdf0a866151f41b8876e73800929933"; };

  };

  class GetModelListResponse : public ros::Msg
  {
    public:
      typedef household_objects_database_msgs::DatabaseReturnCode _return_code_type;
      _return_code_type return_code;
      uint32_t model_ids_length;
      typedef int32_t _model_ids_type;
      _model_ids_type st_model_ids;
      _model_ids_type * model_ids;

    GetModelListResponse():
      return_code(),
      model_ids_length(0), model_ids(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->return_code.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->model_ids_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->model_ids_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->model_ids_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->model_ids_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->model_ids_length);
      for( uint32_t i = 0; i < model_ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_model_idsi;
      u_model_idsi.real = this->model_ids[i];
      *(outbuffer + offset + 0) = (u_model_idsi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_model_idsi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_model_idsi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_model_idsi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->model_ids[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->return_code.deserialize(inbuffer + offset);
      uint32_t model_ids_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      model_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      model_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      model_ids_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->model_ids_length);
      if(model_ids_lengthT > model_ids_length)
        this->model_ids = (int32_t*)realloc(this->model_ids, model_ids_lengthT * sizeof(int32_t));
      model_ids_length = model_ids_lengthT;
      for( uint32_t i = 0; i < model_ids_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_model_ids;
      u_st_model_ids.base = 0;
      u_st_model_ids.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_model_ids.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_model_ids.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_model_ids.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_model_ids = u_st_model_ids.real;
      offset += sizeof(this->st_model_ids);
        memcpy( &(this->model_ids[i]), &(this->st_model_ids), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return GETMODELLIST; };
    const char * getMD5(){ return "81fac8c5d631e612bd9183a923572d53"; };

  };

  class GetModelList {
    public:
    typedef GetModelListRequest Request;
    typedef GetModelListResponse Response;
  };

}
#endif
