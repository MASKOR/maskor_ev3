#ifndef MASKOR_EV3_SKID_STEER_DRIVE_H_
#define MASKOR_EV3_SKID_STEER_DRIVE_H_

#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <gazebo_plugins/gazebo_ros_utils.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

  enum{
    RIGHT_FORK_ARM=0,
    LEFT_FORK_ARM=1,
    FORK_LIFT=2,
    NUM_JOINTS=3
  };

  class Joint;
  class Entity;

  class MaskorEv3ArmPlugin : public ModelPlugin {

  public:
    MaskorEv3ArmPlugin();
    ~MaskorEv3ArmPlugin();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

  protected:
    virtual void UpdateChild();
    virtual void FiniChild();

  private:
    void getForkVelocities();
    void publishJointStates();

    physics::WorldPtr world;
    physics::ModelPtr parent;
    event::ConnectionPtr update_connection_;

    std::string right_fork_joint_name_;
    std::string left_fork_joint_name_;
    std::string fork_lift_joint_name_;
    std::string command_topic_;

    GazeboRosPtr gazebo_ros_;

    double fork_torque;
    double fork_speed_;

    physics::JointPtr joints[NUM_JOINTS];

    //ROS Stuff
    ros::NodeHandle* rosnode_;
    ros::Subscriber cmd_vel_subscriber_;
    std::string tf_prefix_;
    sensor_msgs::JointState joint_state_;
    ros::Publisher joint_state_publisher_;

    boost::mutex lock;

    std::string robot_namespace_;

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    // DiffDrive stuff
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    double z_;
    double rot_;
    bool alive_;
    bool publishJointStates_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;

  };
}

#endif /* MASKOR_EV3_SKID_STEER_DRIVE_H_ */