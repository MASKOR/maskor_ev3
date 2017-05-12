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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo {

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
    void publishOdometry(double step_time);
    void getForkVelocities();

    physics::WorldPtr world;
    physics::ModelPtr parent;
    event::ConnectionPtr update_connection_;

    std::string right_fork_joint_name_;
    std::string left_fork_joint_name_;
    std::string fork_lift_joint_name_;

    double fork_torque;
    double fork_speed_;

    physics::JointPtr joints[3];

    //ROS Stuff
    ros::NodeHandle* rosnode_;
    ros::Subscriber cmd_vel_subscriber_;
    ros::Publisher odometry_publisher_;
    tf::TransformBroadcaster *transform_broadcaster_;
    nav_msgs::Odometry odom_;
    std::string tf_prefix_;
    bool broadcast_tf_;

    boost::mutex lock;

    std::string turtle_name;
    std::string robot_namespace_;
    std::string command_topic_;
    std::string robot_base_frame_;
    std::string odometry_frame_;

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    // DiffDrive stuff
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    double z_;
    double rot_;
    bool alive_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;

    double covariance_x_;
    double covariance_y_;
    double covariance_yaw_;

  };
}

#endif /* MASKOR_EV3_SKID_STEER_DRIVE_H_ */
