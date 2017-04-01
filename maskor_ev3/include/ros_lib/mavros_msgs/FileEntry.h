#ifndef _ROS_mavros_msgs_FileEntry_h
#define _ROS_mavros_msgs_FileEntry_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mavros_msgs
{

  class FileEntry : public ros::Msg
  {
    public:
      const char* name;
      uint8_t type;
      uint64_t size;
      enum { TYPE_FILE =  0 };
      enum { TYPE_DIRECTORY =  1 };

    FileEntry():
      name(""),
      type(0),
      size(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      memcpy(outbuffer + offset, &length_name, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset + 0) = (this->size >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->size >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->size >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->size >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->size >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->size >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->size >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->size >> (8 * 7)) & 0xFF;
      offset += sizeof(this->size);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_name;
      memcpy(&length_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      this->size =  ((uint64_t) (*(inbuffer + offset)));
      this->size |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->size |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->size |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->size |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->size |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->size |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->size |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->size);
     return offset;
    }

    const char * getType(){ return "mavros_msgs/FileEntry"; };
    const char * getMD5(){ return "5ed706bccb946c5b3a5087569cc53ac3"; };

  };

}
#endif