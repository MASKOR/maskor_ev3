#ifndef _ROS_manipulation_msgs_GraspPlanningActionGoal_h
#define _ROS_manipulation_msgs_GraspPlanningActionGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalID.h"
#include "manipulation_msgs/GraspPlanningGoal.h"

namespace manipulation_msgs
{

  class GraspPlanningActionGoal : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalID _goal_id_type;
      _goal_id_type goal_id;
      typedef manipulation_msgs::GraspPlanningGoal _goal_type;
      _goal_type goal;

    GraspPlanningActionGoal():
      header(),
      goal_id(),
      goal()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->goal_id.serialize(outbuffer + offset);
      offset += this->goal.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->goal_id.deserialize(inbuffer + offset);
      offset += this->goal.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "manipulation_msgs/GraspPlanningActionGoal"; };
    const char * getMD5(){ return "51f78205082ab7818f66534367bff5f5"; };

  };

}
#endif