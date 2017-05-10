#include <algorithm>
#include <assert.h>

#include <maskor_ev3_gazebo/maskor_ev3_bobb3e_arm_plugin.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo {

  enum{
    RIGHT_FORK,
    LEFT_FORK,
    FORK_LIFT,
  };

  MaskorEv3ArmPlugin::MaskorEv3ArmPlugin(){}

  MaskorEv3ArmPlugin::~MaskorEv3ArmPlugin(){
    delete rosnode_;
    delete transform_broadcaster_;
  }

  void MaskorEv3ArmPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    //std::cout << "/* message */" << '\n';

    this->parent = _parent;
    this->world = _parent->GetWorld();

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin missing <robotNamespace>, defaults to \"%s\"",
          this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    this->broadcast_tf_ = false;
    if (!_sdf->HasElement("broadcastTF")) {
      if (!this->broadcast_tf_)
    	  ROS_INFO_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <broadcastTF>, defaults to false.",this->robot_namespace_.c_str());
      else ROS_INFO_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <broadcastTF>, defaults to true.",this->robot_namespace_.c_str());

    } else {
      this->broadcast_tf_ = _sdf->GetElement("broadcastTF")->Get<bool>();
    }

    this->left_fork_joint_name_ = "left_fork_joint";
    if (!_sdf->HasElement("leftForkJoint")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <leftForkJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->left_fork_joint_name_.c_str());
    } else {
      this->left_fork_joint_name_ = _sdf->GetElement("leftForkJoint")->Get<std::string>();
    }

    this->right_fork_joint_name_ = "right_fork_joint";
    if (!_sdf->HasElement("rightForkJoint")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <rightForkJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->right_fork_joint_name_.c_str());
    } else {
      this->right_fork_joint_name_ = _sdf->GetElement("rightForkJoint")->Get<std::string>();
    }

    this->fork_lift_joint_name_ = "fork_lift_joint";
    if (!_sdf->HasElement("forkLiftJoint")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <forkLiftJoint>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->fork_lift_joint_name_.c_str());
    } else {
      this->fork_lift_joint_name_ = _sdf->GetElement("forkLiftJoint")->Get<std::string>();
    }

    this->fork_torque = 5.0;
    if (!_sdf->HasElement("forkTorque")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <forkTorque>, defaults to %f",
          this->robot_namespace_.c_str(), this->fork_torque);
    } else {
      this->fork_torque = _sdf->GetElement("forkTorque")->Get<double>();
    }

    this->command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
      this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->robot_base_frame_ = "base_footprint";
    if (!_sdf->HasElement("robotBaseFrame")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
          this->robot_namespace_.c_str(), this->robot_base_frame_.c_str());
    } else {
      this->robot_base_frame_ = _sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <updateRate>, defaults to %f",
          this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
    }

    this->covariance_x_ = 0.0001;
    if (!_sdf->HasElement("covariance_x")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <covariance_x>, defaults to %f",
          this->robot_namespace_.c_str(), covariance_x_);
    } else {
      covariance_x_ = _sdf->GetElement("covariance_x")->Get<double>();
    }

    this->covariance_y_ = 0.0001;
    if (!_sdf->HasElement("covariance_y")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <covariance_y>, defaults to %f",
          this->robot_namespace_.c_str(), covariance_y_);
    } else {
      covariance_y_ = _sdf->GetElement("covariance_y")->Get<double>();
    }

    this->covariance_yaw_ = 0.01;
    if (!_sdf->HasElement("covariance_yaw")) {
      ROS_WARN_NAMED("skid_steer_drive", "GazeboRosSkidSteerDrive Plugin (ns = %s) missing <covariance_yaw>, defaults to %f",
          this->robot_namespace_.c_str(), covariance_yaw_);
    } else {
      covariance_yaw_ = _sdf->GetElement("covariance_yaw")->Get<double>();
    }

    // Initialize update rate stuff
    if (this->update_rate_ > 0.0) {
      this->update_period_ = 1.0 / this->update_rate_;
    } else {
      this->update_period_ = 0.0;
    }
    last_update_time_ = this->world->GetSimTime();

    // Initialize velocity stuff
    fork_speed_ = 0;

    z_ = 0;
    rot_ = 0;
    alive_ = true;

    joints[LEFT_FORK] = this->parent->GetJoint(left_fork_joint_name_);
    joints[RIGHT_FORK] = this->parent->GetJoint(right_fork_joint_name_);
    joints[FORK_LIFT] = this->parent->GetJoint(fork_lift_joint_name_);

    #if GAZEBO_MAJOR_VERSION > 2
        joints[LEFT_FORK]->SetParam("fmax", 0, fork_torque);
        joints[RIGHT_FORK]->SetParam("fmax", 0, fork_torque);
        joints[FORK_LIFT]->SetParam("fmax", 0, fork_torque);
    #else
        joints[LEFT_FORK]->SetMaxForce(0, fork_torque);
        joints[RIGHT_FORK]->SetMaxForce(0, fork_torque);
        joints[FORK_LIFT]->SetMaxForce(0, fork_torque);
    #endif

    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM_NAMED("skid_steer_drive", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    ROS_INFO_NAMED("skid_steer_drive", "Starting GazeboRosSkidSteerDrive Plugin (ns = %s)", this->robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);
    transform_broadcaster_ = new tf::TransformBroadcaster();

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&MaskorEv3ArmPlugin::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = rosnode_->subscribe(so);



    // start custom queue for diff drive
    this->callback_queue_thread_ =
      boost::thread(boost::bind(&MaskorEv3ArmPlugin::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MaskorEv3ArmPlugin::UpdateChild, this));
  }

  // Update the controller
  void MaskorEv3ArmPlugin::UpdateChild() {
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update =
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {

      publishOdometry(seconds_since_last_update);

      // Update robot in case new velocities have been requested
      getForkVelocities();
      #if GAZEBO_MAJOR_VERSION > 2
            joints[LEFT_FORK]->SetParam("vel", 0, (fork_speed_*3) * -1);
            joints[RIGHT_FORK]->SetParam("vel", 0, (fork_speed_*3) * -1);
            joints[FORK_LIFT]->SetParam("vel", 0, fork_speed_);
      #else
            joints[LEFT_FORK]->SetVelocity(0, fork_speed_);
            joints[RIGHT_FORK]->SetVelocity(0, fork_speed_);
            joints[FORK_LIFT]->SetVelocity(0, fork_speed_);
      #endif

        last_update_time_+= common::Time(update_period_);
      }
    }

    // Finalize the controller
    void MaskorEv3ArmPlugin::FiniChild() {
      alive_ = false;
      queue_.clear();
      queue_.disable();
      rosnode_->shutdown();
      callback_queue_thread_.join();
    }

    void MaskorEv3ArmPlugin::getForkVelocities() {
      boost::mutex::scoped_lock scoped_lock(lock);

      double vh = z_;

      fork_speed_ = z_;

    }

    void MaskorEv3ArmPlugin::cmdVelCallback(
        const geometry_msgs::Twist::ConstPtr& cmd_msg) {

      boost::mutex::scoped_lock scoped_lock(lock);
      z_ = cmd_msg->linear.z;


    }

    void MaskorEv3ArmPlugin::QueueThread() {
      static const double timeout = 0.01;

      while (alive_ && rosnode_->ok()) {
        queue_.callAvailable(ros::WallDuration(timeout));
      }
    }

    void MaskorEv3ArmPlugin::publishOdometry(double step_time) {
      ros::Time current_time = ros::Time::now();
      std::string odom_frame =
        tf::resolve(tf_prefix_, odometry_frame_);
      std::string base_footprint_frame =
        tf::resolve(tf_prefix_, robot_base_frame_);

      // TODO create some non-perfect odometry!
      // getting data for base_footprint to odom transform
      math::Pose pose = this->parent->GetWorldPose();

      tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
      tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

      tf::Transform base_footprint_to_odom(qt, vt);
      if (this->broadcast_tf_) {

      	transform_broadcaster_->sendTransform(
          tf::StampedTransform(base_footprint_to_odom, current_time,
              odom_frame, base_footprint_frame));

      }

    /*  // publish odom topic
      odom_.pose.pose.position.x = pose.pos.x;
      odom_.pose.pose.position.y = pose.pos.y;

      odom_.pose.pose.orientation.x = pose.rot.x;
      odom_.pose.pose.orientation.y = pose.rot.y;
      odom_.pose.pose.orientation.z = pose.rot.z;
      odom_.pose.pose.orientation.w = pose.rot.w;
      odom_.pose.covariance[0] = this->covariance_x_;
      odom_.pose.covariance[7] = this->covariance_y_;
      odom_.pose.covariance[14] = 1000000000000.0;
      odom_.pose.covariance[21] = 1000000000000.0;
      odom_.pose.covariance[28] = 1000000000000.0;
      odom_.pose.covariance[35] = this->covariance_yaw_;

      // get velocity in /odom frame
      math::Vector3 linear;
      linear = this->parent->GetWorldLinearVel();
      odom_.twist.twist.angular.z = this->parent->GetWorldAngularVel().z;

      // convert velocity to child_frame_id (aka base_footprint)
      float yaw = pose.rot.GetYaw();
      odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
      odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;
      odom_.twist.covariance[0] = this->covariance_x_;
      odom_.twist.covariance[7] = this->covariance_y_;
      odom_.twist.covariance[14] = 1000000000000.0;
      odom_.twist.covariance[21] = 1000000000000.0;
      odom_.twist.covariance[28] = 1000000000000.0;
      odom_.twist.covariance[35] = this->covariance_yaw_;

      odom_.header.stamp = current_time;
      odom_.header.frame_id = odom_frame;
      odom_.child_frame_id = base_footprint_frame;


*/
    odometry_publisher_.publish(odom_);
    }

    GZ_REGISTER_MODEL_PLUGIN(MaskorEv3ArmPlugin)
}
