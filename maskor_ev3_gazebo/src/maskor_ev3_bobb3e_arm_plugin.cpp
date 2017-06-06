#include <algorithm>
#include <assert.h>

#include <maskor_ev3_gazebo/maskor_ev3_bobb3e_arm_plugin.h>

#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

namespace gazebo {

  MaskorEv3ArmPlugin::MaskorEv3ArmPlugin(){}

  MaskorEv3ArmPlugin::~MaskorEv3ArmPlugin(){
    delete rosnode_;
  }

  void MaskorEv3ArmPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    this->parent = _parent;
    this->world = _parent->GetWorld();

    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "maskor_ev3_bobb3e_arm_plugin" ) );
    gazebo_ros_->isInitialized();

    this->publishJointStates_=true;
    if (publishJointStates_) {
      joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
      ROS_INFO_NAMED("MaskorEV3Bobb3eArmPlugin", "%s: Advertise joint_states!", gazebo_ros_->info());
    }

    this->robot_namespace_ = "";
    if (!_sdf->HasElement("robotNamespace")) {
      ROS_INFO_NAMED("MaskorEV3Bobb3eArmPlugin", "MaskorEV3Bobb3eArmPlugin Plugin missing <robotNamespace>, defaults to \"%s\"",
		     this->robot_namespace_.c_str());
    } else {
      this->robot_namespace_ =
        _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
    }

    this->left_fork_joint_name_ = "left_fork_joint";
    if (!_sdf->HasElement("leftForkJoint")) {
      ROS_WARN_NAMED("MaskorEV3Bobb3eArmPlugin", "MaskorEV3Bobb3eArmPlugin Plugin (ns = %s) missing <leftForkJoint>, defaults to \"%s\"",
		     this->robot_namespace_.c_str(), this->left_fork_joint_name_.c_str());
    } else {
      this->left_fork_joint_name_ = _sdf->GetElement("leftForkJoint")->Get<std::string>();
    }

    this->right_fork_joint_name_ = "right_fork_joint";
    if (!_sdf->HasElement("rightForkJoint")) {
      ROS_WARN_NAMED("MaskorEV3Bobb3eArmPlugin", "MaskorEV3Bobb3eArmPlugin Plugin (ns = %s) missing <rightForkJoint>, defaults to \"%s\"",
		     this->robot_namespace_.c_str(), this->right_fork_joint_name_.c_str());
    } else {
      this->right_fork_joint_name_ = _sdf->GetElement("rightForkJoint")->Get<std::string>();
    }

    this->fork_lift_joint_name_ = "fork_lift_joint";
    if (!_sdf->HasElement("forkLiftJoint")) {
      ROS_WARN_NAMED("MaskorEV3Bobb3eArmPlugin", "MaskorEV3Bobb3eArmPlugin Plugin (ns = %s) missing <forkLiftJoint>, defaults to \"%s\"",
		     this->robot_namespace_.c_str(), this->fork_lift_joint_name_.c_str());
    } else {
      this->fork_lift_joint_name_ = _sdf->GetElement("forkLiftJoint")->Get<std::string>();
    }

    this->fork_torque = 5.0;
    if (!_sdf->HasElement("forkTorque")) {
      ROS_WARN_NAMED("MaskorEV3Bobb3eArmPlugin", "MaskorEV3Bobb3eArmPlugin Plugin (ns = %s) missing <forkTorque>, defaults to %f",
		     this->robot_namespace_.c_str(), this->fork_torque);
    } else {
      this->fork_torque = _sdf->GetElement("forkTorque")->Get<double>();
    }

    this->command_topic_ = "cmd_vel";
    if (!_sdf->HasElement("commandTopic")) {
      ROS_WARN_NAMED("MaskorEV3Bobb3eArmPlugin", "MaskorEV3Bobb3eArmPlugin Plugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
		     this->robot_namespace_.c_str(), this->command_topic_.c_str());
    } else {
      this->command_topic_ = _sdf->GetElement("commandTopic")->Get<std::string>();
    }

    this->update_rate_ = 100.0;
    if (!_sdf->HasElement("updateRate")) {
      ROS_WARN_NAMED("MaskorEV3Bobb3eArmPlugin", "MaskorEV3Bobb3eArmPlugin Plugin (ns = %s) missing <updateRate>, defaults to %f",
		     this->robot_namespace_.c_str(), this->update_rate_);
    } else {
      this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
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

    joints[LEFT_FORK_ARM] = this->parent->GetJoint(left_fork_joint_name_);
    joints[RIGHT_FORK_ARM] = this->parent->GetJoint(right_fork_joint_name_);
    joints[FORK_LIFT] = this->parent->GetJoint(fork_lift_joint_name_);

#if GAZEBO_MAJOR_VERSION > 2
    joints[LEFT_FORK_ARM]->SetParam("fmax", 0, fork_torque);
    joints[RIGHT_FORK_ARM]->SetParam("fmax", 0, fork_torque);
    joints[FORK_LIFT]->SetParam("fmax", 0, fork_torque);
#else
    joints[LEFT_FORK_ARM]->SetMaxForce(0, fork_torque);
    joints[RIGHT_FORK_ARM]->SetMaxForce(0, fork_torque);
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

    ROS_INFO_NAMED("", "Starting MaskorEV3Bobb3eArmPlugin Plugin (ns = %s)", this->robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);

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

  void MaskorEv3ArmPlugin::publishJointStates()
  {
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( NUM_JOINTS );
    joint_state_.position.resize ( NUM_JOINTS );

    for ( int i = 0; i < NUM_JOINTS; i++ ) {
      physics::JointPtr joint = joints[i];
      math::Angle angle = joint->GetAngle ( 0 );
      joint_state_.name[i] = joint->GetName();
      joint_state_.position[i] = angle.Radian () ;
    }
    joint_state_publisher_.publish ( joint_state_ );
  }

  // Update the controller
  void MaskorEv3ArmPlugin::UpdateChild() {
    common::Time current_time = this->world->GetSimTime();
    double seconds_since_last_update =
      (current_time - last_update_time_).Double();
    if (seconds_since_last_update > update_period_) {
      
      // Update robot in case new velocities have been requested
      getForkVelocities();
      
#if GAZEBO_MAJOR_VERSION > 2
      joints[LEFT_FORK_ARM]->SetParam("vel", 0, (fork_speed_*3) * -1);
      joints[RIGHT_FORK_ARM]->SetParam("vel", 0, (fork_speed_*3) * -1);
      joints[FORK_LIFT]->SetParam("vel", 0, fork_speed_);
#else
      joints[LEFT_FORK_ARM]->SetVelocity(0, fork_speed_);
      joints[RIGHT_FORK_ARM]->SetVelocity(0, fork_speed_);
      joints[FORK_LIFT]->SetVelocity(0, fork_speed_);
#endif

      publishJointStates();
	    
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

}

GZ_REGISTER_MODEL_PLUGIN(MaskorEv3ArmPlugin)
}
