#ifndef _ROS_SERVICE_GraspPlanning_h
#define _ROS_SERVICE_GraspPlanning_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "manipulation_msgs/GraspPlanningErrorCode.h"
#include "manipulation_msgs/GraspableObject.h"
#include "manipulation_msgs/Grasp.h"

namespace manipulation_msgs
{

static const char GRASPPLANNING[] = "manipulation_msgs/GraspPlanning";

  class GraspPlanningRequest : public ros::Msg
  {
    public:
      typedef const char* _arm_name_type;
      _arm_name_type arm_name;
      typedef manipulation_msgs::GraspableObject _target_type;
      _target_type target;
      typedef const char* _collision_object_name_type;
      _collision_object_name_type collision_object_name;
      typedef const char* _collision_support_surface_name_type;
      _collision_support_surface_name_type collision_support_surface_name;
      uint32_t grasps_to_evaluate_length;
      typedef manipulation_msgs::Grasp _grasps_to_evaluate_type;
      _grasps_to_evaluate_type st_grasps_to_evaluate;
      _grasps_to_evaluate_type * grasps_to_evaluate;
      uint32_t movable_obstacles_length;
      typedef manipulation_msgs::GraspableObject _movable_obstacles_type;
      _movable_obstacles_type st_movable_obstacles;
      _movable_obstacles_type * movable_obstacles;

    GraspPlanningRequest():
      arm_name(""),
      target(),
      collision_object_name(""),
      collision_support_surface_name(""),
      grasps_to_evaluate_length(0), grasps_to_evaluate(NULL),
      movable_obstacles_length(0), movable_obstacles(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_arm_name = strlen(this->arm_name);
      varToArr(outbuffer + offset, length_arm_name);
      offset += 4;
      memcpy(outbuffer + offset, this->arm_name, length_arm_name);
      offset += length_arm_name;
      offset += this->target.serialize(outbuffer + offset);
      uint32_t length_collision_object_name = strlen(this->collision_object_name);
      varToArr(outbuffer + offset, length_collision_object_name);
      offset += 4;
      memcpy(outbuffer + offset, this->collision_object_name, length_collision_object_name);
      offset += length_collision_object_name;
      uint32_t length_collision_support_surface_name = strlen(this->collision_support_surface_name);
      varToArr(outbuffer + offset, length_collision_support_surface_name);
      offset += 4;
      memcpy(outbuffer + offset, this->collision_support_surface_name, length_collision_support_surface_name);
      offset += length_collision_support_surface_name;
      *(outbuffer + offset + 0) = (this->grasps_to_evaluate_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->grasps_to_evaluate_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->grasps_to_evaluate_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->grasps_to_evaluate_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->grasps_to_evaluate_length);
      for( uint32_t i = 0; i < grasps_to_evaluate_length; i++){
      offset += this->grasps_to_evaluate[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->movable_obstacles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->movable_obstacles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->movable_obstacles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->movable_obstacles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->movable_obstacles_length);
      for( uint32_t i = 0; i < movable_obstacles_length; i++){
      offset += this->movable_obstacles[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_arm_name;
      arrToVar(length_arm_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_arm_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_arm_name-1]=0;
      this->arm_name = (char *)(inbuffer + offset-1);
      offset += length_arm_name;
      offset += this->target.deserialize(inbuffer + offset);
      uint32_t length_collision_object_name;
      arrToVar(length_collision_object_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_collision_object_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_collision_object_name-1]=0;
      this->collision_object_name = (char *)(inbuffer + offset-1);
      offset += length_collision_object_name;
      uint32_t length_collision_support_surface_name;
      arrToVar(length_collision_support_surface_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_collision_support_surface_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_collision_support_surface_name-1]=0;
      this->collision_support_surface_name = (char *)(inbuffer + offset-1);
      offset += length_collision_support_surface_name;
      uint32_t grasps_to_evaluate_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      grasps_to_evaluate_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      grasps_to_evaluate_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      grasps_to_evaluate_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->grasps_to_evaluate_length);
      if(grasps_to_evaluate_lengthT > grasps_to_evaluate_length)
        this->grasps_to_evaluate = (manipulation_msgs::Grasp*)realloc(this->grasps_to_evaluate, grasps_to_evaluate_lengthT * sizeof(manipulation_msgs::Grasp));
      grasps_to_evaluate_length = grasps_to_evaluate_lengthT;
      for( uint32_t i = 0; i < grasps_to_evaluate_length; i++){
      offset += this->st_grasps_to_evaluate.deserialize(inbuffer + offset);
        memcpy( &(this->grasps_to_evaluate[i]), &(this->st_grasps_to_evaluate), sizeof(manipulation_msgs::Grasp));
      }
      uint32_t movable_obstacles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      movable_obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      movable_obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      movable_obstacles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->movable_obstacles_length);
      if(movable_obstacles_lengthT > movable_obstacles_length)
        this->movable_obstacles = (manipulation_msgs::GraspableObject*)realloc(this->movable_obstacles, movable_obstacles_lengthT * sizeof(manipulation_msgs::GraspableObject));
      movable_obstacles_length = movable_obstacles_lengthT;
      for( uint32_t i = 0; i < movable_obstacles_length; i++){
      offset += this->st_movable_obstacles.deserialize(inbuffer + offset);
        memcpy( &(this->movable_obstacles[i]), &(this->st_movable_obstacles), sizeof(manipulation_msgs::GraspableObject));
      }
     return offset;
    }

    const char * getType(){ return GRASPPLANNING; };
    const char * getMD5(){ return "077dca08a07415d82d6ab047890161f4"; };

  };

  class GraspPlanningResponse : public ros::Msg
  {
    public:
      uint32_t grasps_length;
      typedef manipulation_msgs::Grasp _grasps_type;
      _grasps_type st_grasps;
      _grasps_type * grasps;
      typedef manipulation_msgs::GraspPlanningErrorCode _error_code_type;
      _error_code_type error_code;

    GraspPlanningResponse():
      grasps_length(0), grasps(NULL),
      error_code()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->grasps_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->grasps_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->grasps_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->grasps_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->grasps_length);
      for( uint32_t i = 0; i < grasps_length; i++){
      offset += this->grasps[i].serialize(outbuffer + offset);
      }
      offset += this->error_code.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t grasps_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      grasps_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      grasps_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      grasps_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->grasps_length);
      if(grasps_lengthT > grasps_length)
        this->grasps = (manipulation_msgs::Grasp*)realloc(this->grasps, grasps_lengthT * sizeof(manipulation_msgs::Grasp));
      grasps_length = grasps_lengthT;
      for( uint32_t i = 0; i < grasps_length; i++){
      offset += this->st_grasps.deserialize(inbuffer + offset);
        memcpy( &(this->grasps[i]), &(this->st_grasps), sizeof(manipulation_msgs::Grasp));
      }
      offset += this->error_code.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GRASPPLANNING; };
    const char * getMD5(){ return "ff7a88c4aec40207164535a24dc9c440"; };

  };

  class GraspPlanning {
    public:
    typedef GraspPlanningRequest Request;
    typedef GraspPlanningResponse Response;
  };

}
#endif
