#ifndef _ROS_manipulation_msgs_GraspableObjectList_h
#define _ROS_manipulation_msgs_GraspableObjectList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "manipulation_msgs/GraspableObject.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "shape_msgs/Mesh.h"
#include "geometry_msgs/Pose.h"

namespace manipulation_msgs
{

  class GraspableObjectList : public ros::Msg
  {
    public:
      uint32_t graspable_objects_length;
      typedef manipulation_msgs::GraspableObject _graspable_objects_type;
      _graspable_objects_type st_graspable_objects;
      _graspable_objects_type * graspable_objects;
      typedef sensor_msgs::Image _image_type;
      _image_type image;
      typedef sensor_msgs::CameraInfo _camera_info_type;
      _camera_info_type camera_info;
      uint32_t meshes_length;
      typedef shape_msgs::Mesh _meshes_type;
      _meshes_type st_meshes;
      _meshes_type * meshes;
      typedef geometry_msgs::Pose _reference_to_camera_type;
      _reference_to_camera_type reference_to_camera;

    GraspableObjectList():
      graspable_objects_length(0), graspable_objects(NULL),
      image(),
      camera_info(),
      meshes_length(0), meshes(NULL),
      reference_to_camera()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->graspable_objects_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->graspable_objects_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->graspable_objects_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->graspable_objects_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->graspable_objects_length);
      for( uint32_t i = 0; i < graspable_objects_length; i++){
      offset += this->graspable_objects[i].serialize(outbuffer + offset);
      }
      offset += this->image.serialize(outbuffer + offset);
      offset += this->camera_info.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->meshes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->meshes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->meshes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->meshes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->meshes_length);
      for( uint32_t i = 0; i < meshes_length; i++){
      offset += this->meshes[i].serialize(outbuffer + offset);
      }
      offset += this->reference_to_camera.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t graspable_objects_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      graspable_objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      graspable_objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      graspable_objects_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->graspable_objects_length);
      if(graspable_objects_lengthT > graspable_objects_length)
        this->graspable_objects = (manipulation_msgs::GraspableObject*)realloc(this->graspable_objects, graspable_objects_lengthT * sizeof(manipulation_msgs::GraspableObject));
      graspable_objects_length = graspable_objects_lengthT;
      for( uint32_t i = 0; i < graspable_objects_length; i++){
      offset += this->st_graspable_objects.deserialize(inbuffer + offset);
        memcpy( &(this->graspable_objects[i]), &(this->st_graspable_objects), sizeof(manipulation_msgs::GraspableObject));
      }
      offset += this->image.deserialize(inbuffer + offset);
      offset += this->camera_info.deserialize(inbuffer + offset);
      uint32_t meshes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      meshes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      meshes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      meshes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->meshes_length);
      if(meshes_lengthT > meshes_length)
        this->meshes = (shape_msgs::Mesh*)realloc(this->meshes, meshes_lengthT * sizeof(shape_msgs::Mesh));
      meshes_length = meshes_lengthT;
      for( uint32_t i = 0; i < meshes_length; i++){
      offset += this->st_meshes.deserialize(inbuffer + offset);
        memcpy( &(this->meshes[i]), &(this->st_meshes), sizeof(shape_msgs::Mesh));
      }
      offset += this->reference_to_camera.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "manipulation_msgs/GraspableObjectList"; };
    const char * getMD5(){ return "d67571f2982f1b7115de1e0027319109"; };

  };

}
#endif