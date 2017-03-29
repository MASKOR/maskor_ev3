#ifndef _ROS_manipulation_msgs_GraspPlanningAction_h
#define _ROS_manipulation_msgs_GraspPlanningAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "manipulation_msgs/GraspPlanningActionGoal.h"
#include "manipulation_msgs/GraspPlanningActionResult.h"
#include "manipulation_msgs/GraspPlanningActionFeedback.h"

namespace manipulation_msgs
{

  class GraspPlanningAction : public ros::Msg
  {
    public:
      typedef manipulation_msgs::GraspPlanningActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef manipulation_msgs::GraspPlanningActionResult _action_result_type;
      _action_result_type action_result;
      typedef manipulation_msgs::GraspPlanningActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    GraspPlanningAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "manipulation_msgs/GraspPlanningAction"; };
    const char * getMD5(){ return "f2d7ee4c83f481d31e151ec1b1f209b4"; };

  };

}
#endif