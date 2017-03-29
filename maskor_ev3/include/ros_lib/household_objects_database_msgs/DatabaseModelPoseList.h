#ifndef _ROS_household_objects_database_msgs_DatabaseModelPoseList_h
#define _ROS_household_objects_database_msgs_DatabaseModelPoseList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "household_objects_database_msgs/DatabaseModelPose.h"

namespace household_objects_database_msgs
{

  class DatabaseModelPoseList : public ros::Msg
  {
    public:
      uint32_t model_list_length;
      typedef household_objects_database_msgs::DatabaseModelPose _model_list_type;
      _model_list_type st_model_list;
      _model_list_type * model_list;

    DatabaseModelPoseList():
      model_list_length(0), model_list(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->model_list_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->model_list_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->model_list_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->model_list_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->model_list_length);
      for( uint32_t i = 0; i < model_list_length; i++){
      offset += this->model_list[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t model_list_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      model_list_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      model_list_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      model_list_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->model_list_length);
      if(model_list_lengthT > model_list_length)
        this->model_list = (household_objects_database_msgs::DatabaseModelPose*)realloc(this->model_list, model_list_lengthT * sizeof(household_objects_database_msgs::DatabaseModelPose));
      model_list_length = model_list_lengthT;
      for( uint32_t i = 0; i < model_list_length; i++){
      offset += this->st_model_list.deserialize(inbuffer + offset);
        memcpy( &(this->model_list[i]), &(this->st_model_list), sizeof(household_objects_database_msgs::DatabaseModelPose));
      }
     return offset;
    }

    const char * getType(){ return "household_objects_database_msgs/DatabaseModelPoseList"; };
    const char * getMD5(){ return "f0bb2aa8d2ededee0ffe0f5d2107e099"; };

  };

}
#endif