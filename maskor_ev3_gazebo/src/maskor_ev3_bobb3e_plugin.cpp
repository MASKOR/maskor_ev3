/*

  MASKOR EV3 BOBB3E PLUGIN
  Copyright (c) 2017
  FH Aachen University of Applied Sciences

  Marcel Stüttgen
  stuettgen@fh-aachen.de
  Dennis Miltz
  dennis.miltz@alumni.fh-aachen.de
  Christoph Gollol
  christoph.gollok@alumni.fh-aachen.de


  based on: Gazebo Diff Drive Plugin

  Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
  All rights reserved.
  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
  * Redistributions of source code must retain the above copyright
  notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above copyright
  notice, this list of conditions and the following disclaimer in the
  documentation and/or other materials provided with the distribution.
  * Neither the name of the <organization> nor the
  names of its contributors may be used to endorse or promote products
  derived from this software without specific prior written permission.
  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <algorithm>
#include <assert.h>
#include <algorithm>
#include <assert.h>

#include <maskor_ev3_gazebo/maskor_ev3_bobb3e_plugin.h>
#include <gazebo/math/gzmath.hh>
#include <sdf/sdf.hh>
#include <ros/ros.h>

namespace gazebo
{

    enum JointID{
      RIGHT_FRONT_WHEEL,
      RIGHT_REAR_WHEEL,
      LEFT_FRONT_WHEEL,
      LEFT_REAR_WHEEL,
      LEFT_ARM,
      RIGHT_ARM,
      FORK_LIFT,
      NUM_JOINTS
    };

  MaskorEV3Bobb3ePlugin::MaskorEV3Bobb3ePlugin() {}

  // Destructor
  MaskorEV3Bobb3ePlugin::~MaskorEV3Bobb3ePlugin() {}

  // Load the controller
  void MaskorEV3Bobb3ePlugin::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
  {

    this->parent = _parent;
    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( _parent, _sdf, "maskor_ev3_bobb3e_plugin" ) );
    // Make sure the ROS node for Gazebo has already been initialized
    gazebo_ros_->isInitialized();

    gazebo_ros_->getParameter<std::string> ( command_topic_, "commandTopic", "cmd_vel" );
    gazebo_ros_->getParameter<std::string> ( odometry_topic_, "odometryTopic", "odom" );
    gazebo_ros_->getParameter<std::string> ( odometry_frame_, "odometryFrame", "odom" );
    gazebo_ros_->getParameter<std::string> ( robot_base_frame_, "robotBaseFrame", "base_footprint" );
    gazebo_ros_->getParameterBoolean ( publishWheelTF_, "publishWheelTF", false );
    gazebo_ros_->getParameterBoolean ( publishWheelJointState_, "publishWheelJointState", false );
    gazebo_ros_->getParameterBoolean ( legacy_mode_, "legacyMode", false );

    if (!_sdf->HasElement("legacyMode"))
      {
	ROS_ERROR_NAMED("MaskorEV3Bobb3ePlugin", "MaskorEV3Bobb3ePlugin Plugin missing <legacyMode>, defaults to true\n"
			"This setting assumes you have a old package, where the right and left wheel are changed to fix a former code issue\n"
			"To get rid of this error just set <legacyMode> to false if you just created a new package.\n"
			"To fix an old package you have to exchange left wheel by the right wheel.\n"
			"If you do not want to fix this issue in an old package or your z axis points down instead of the ROS standard defined in REP 103\n"
			"just set <legacyMode> to true.\n"
			);
      }

    gazebo_ros_->getParameter<double> ( wheel_separation_, "wheelSeparation", 0.095 );
    gazebo_ros_->getParameter<double> ( wheel_diameter_, "wheelDiameter", 0.03 );
    gazebo_ros_->getParameter<double> ( wheel_accel, "wheelAcceleration", 1.0 );
    gazebo_ros_->getParameter<double> ( wheel_torque, "wheelTorque", 1.0 );
    gazebo_ros_->getParameter<double> ( update_rate_, "updateRate", 30.0 );
    std::map<std::string, OdomSource> odomOptions;
    odomOptions["encoder"] = ENCODER;
    odomOptions["world"] = WORLD;
    gazebo_ros_->getParameter<OdomSource> ( odom_source_, "odometrySource", odomOptions, WORLD );


    // ROS_INFO_NAMED("***** LOADED PARAMETERS *****");
    ROS_INFO_NAMED("Param wheelSeparation","wheelSeparation: \t\t%f", wheel_separation_);
    ROS_INFO_NAMED("Param wheelDiameter","wheelDiameter: \t\t%f", wheel_diameter_);
    ROS_INFO_NAMED("Param wheelAcceleration","wheelAcceleration: \t%f", wheel_accel);
    ROS_INFO_NAMED("Param wheelTorque","wheelTorque: \t\t%f", wheel_torque);
    ROS_INFO_NAMED("Param updateRate","updateRate: \t\t%f", update_rate_);


    gazebo_ros_->getParameter<std::string> (front_left_wheel_,"frontLeftWheelJoint", "base_link_to_left_front_wheel");
    gazebo_ros_->getParameter<std::string> (front_right_wheel_,"frontRightWheelJoint", "base_link_to_right_front_wheel");
    gazebo_ros_->getParameter<std::string> (rear_left_wheel_,"rearLeftWheelJoint", "base_link_to_left_rear_wheel");
    gazebo_ros_->getParameter<std::string> (rear_right_wheel_,"rearRightWheelJoint", "base_link_to_right_rear_wheel");


    joints_.resize ( NUM_JOINTS );
    joints_[RIGHT_FRONT_WHEEL] = gazebo_ros_->getJoint ( parent, front_right_wheel_.c_str(), "base_link_to_right_front_wheel" );
    joints_[LEFT_FRONT_WHEEL] = gazebo_ros_->getJoint ( parent, front_left_wheel_.c_str(), "base_link_to_left_front_wheel" );
    joints_[LEFT_REAR_WHEEL] = gazebo_ros_->getJoint ( parent, rear_left_wheel_.c_str(),  "base_link_to_left_rear_wheel");
    joints_[RIGHT_REAR_WHEEL] = gazebo_ros_->getJoint ( parent, rear_right_wheel_.c_str(),  "base_link_to_right_rear_wheel");
    joints_[LEFT_ARM] = gazebo_ros_->getJoint ( parent, rear_right_wheel_.c_str(),  "base_link_to_left_fork_arm");
    joints_[RIGHT_ARM] = gazebo_ros_->getJoint ( parent, rear_right_wheel_.c_str(),  "base_link_to_right_fork_arm");
    joints_[FORK_LIFT] = gazebo_ros_->getJoint ( parent, rear_right_wheel_.c_str(),  "base_link_to_fork_lift_link");


#if GAZEBO_MAJOR_VERSION > 2
    joints_[RIGHT_FRONT_WHEEL]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_REAR_WHEEL]->SetParam ( "fmax", 0, wheel_torque );
    joints_[LEFT_FRONT_WHEEL]->SetParam ( "fmax", 0, wheel_torque );
    joints_[LEFT_REAR_WHEEL]->SetParam ( "fmax", 0, wheel_torque );
    joints_[LEFT_ARM]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_ARM]->SetParam ( "fmax", 0, wheel_torque );
    joints_[FORK_LIFT]->SetParam ( "fmax", 0, wheel_torque );

#else
    joints_[RIGHT_FRONT_WHEEL]->SetMaxForce ( 0, wheel_torque );
    joints_[RIGHT_REAR_WHEEL]->SetMaxForce ( 0, wheel_torque );
    joints_[LEFT_FRONT_WHEEL]->SetMaxForce ( 0, wheel_torque );
    joints_[LEFT_REAR_WHEEL]->SetMaxForce ( 0, wheel_torque );
    joints_[LEFT_ARM]->SetMaxForce ( 0, wheel_torque );
    joints_[RIGHT_ARM]->SetMaxForce ( 0, wheel_torque );
    joints_[FORK_LIFT]->SetMaxForce ( 0, wheel_torque );
#endif


    this->publish_odom_tf_ = true;
    if (!_sdf->HasElement("publishTf")) {
      ROS_WARN_NAMED("diff_drive", "MaskorEV3Bobb3ePlugin Plugin (ns = %s) missing <publishTf>, defaults to %d",
		     this->robot_namespace_.c_str(), this->publish_odom_tf_);
    } else {
      this->publish_odom_tf_ = _sdf->GetElement("publishTf")->Get<bool>();
    }

    // Initialize update rate stuff
    if ( this->update_rate_ > 0.0 ) this->update_period_ = 1.0 / this->update_rate_;
    else this->update_period_ = 0.0;
    last_update_time_ = parent->GetWorld()->GetSimTime();

    // Initialize velocity stuff
    joint_speeds_[RIGHT_FRONT_WHEEL] = 0;
    joint_speeds_[RIGHT_REAR_WHEEL] = 0;
    joint_speeds_[LEFT_FRONT_WHEEL] = 0;
    joint_speeds_[LEFT_REAR_WHEEL] = 0;
    joint_speeds_[LEFT_ARM] = 5;
    joint_speeds_[RIGHT_ARM] = 10;
    joint_speeds_[FORK_LIFT] = 0;

    // Initialize velocity support stuff
    joint_speeds_instr_[RIGHT_FRONT_WHEEL] = 0;
    joint_speeds_instr_[RIGHT_FRONT_WHEEL] = 0;
    joint_speeds_instr_[LEFT_FRONT_WHEEL] = 0;
    joint_speeds_instr_[LEFT_REAR_WHEEL] = 0;
    joint_speeds_instr_[LEFT_ARM] = 0;
    joint_speeds_instr_[RIGHT_ARM] = 0;
    joint_speeds_instr_[FORK_LIFT] = 0;

    x_ = 0;
    z_ = 0;
    rot_ = 0;
    alive_ = true;


    if (this->publishWheelJointState_)
      {
        joint_state_publisher_ = gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 1000);
        ROS_INFO_NAMED("MaskorEV3Bobb3ePlugin", "%s: Advertise joint_states!", gazebo_ros_->info());
      }

    transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

    // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
    ROS_INFO_NAMED("MaskorEV3Bobb3ePlugin", "%s: Try to subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
							  boost::bind(&MaskorEV3Bobb3ePlugin::cmdVelCallback, this, _1),
							  ros::VoidPtr(), &queue_);

    cmd_vel_subscriber_ = gazebo_ros_->node()->subscribe(so);
    ROS_INFO_NAMED("MaskorEV3Bobb3ePlugin", "%s: Subscribe to %s!", gazebo_ros_->info(), command_topic_.c_str());

    if (this->publish_odom_tf_)
      {
	odometry_publisher_ = gazebo_ros_->node()->advertise<nav_msgs::Odometry>(odometry_topic_, 1);
	ROS_INFO_NAMED("MaskorEV3Bobb3ePlugin", "%s: Advertise odom on %s !", gazebo_ros_->info(), odometry_topic_.c_str());
      }

    // start custom queue for diff drive
    this->callback_queue_thread_ =
      boost::thread ( boost::bind ( &MaskorEV3Bobb3ePlugin::QueueThread, this ) );

    // listen to the update event (broadcast every simulation iteration)
    this->update_connection_ =
      event::Events::ConnectWorldUpdateBegin ( boost::bind ( &MaskorEV3Bobb3ePlugin::UpdateChild, this ) );

  }
  void MaskorEV3Bobb3ePlugin::Reset()
  {
    last_update_time_ = parent->GetWorld()->GetSimTime();
    pose_encoder_.x = 0;
    pose_encoder_.y = 0;
    pose_encoder_.theta = 0;
    x_ = 0;
    z_ = 0;
    rot_ = 0;
#if GAZEBO_MAJOR_VERSION > 2
    joints_[RIGHT_FRONT_WHEEL]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_REAR_WHEEL]->SetParam ( "fmax", 0, wheel_torque );
    joints_[LEFT_FRONT_WHEEL]->SetParam ( "fmax", 0, wheel_torque );
    joints_[LEFT_REAR_WHEEL]->SetParam ( "fmax", 0, wheel_torque );
    joints_[LEFT_ARM]->SetParam ( "fmax", 0, wheel_torque );
    joints_[RIGHT_ARM]->SetParam ( "fmax", 0, wheel_torque );
    joints_[FORK_LIFT]->SetParam ( "fmax", 0, wheel_torque );

#else
    joints_[RIGHT_FRONT_WHEEL]->SetMaxForce ( 0, wheel_torque );
    joints_[RIGHT_REAR_WHEEL]->SetMaxForce ( 0, wheel_torque );
    joints_[LEFT_FRONT_WHEEL]->SetMaxForce ( 0, wheel_torque );
    joints_[LEFT_REAR_WHEEL]->SetMaxForce ( 0, wheel_torque );
    joints_[LEFT_ARM]->SetMaxForce ( 0, wheel_torque );
    joints_[RIGHT_ARM]->SetMaxForce ( 0, wheel_torque );
    joints_[FORK_LIFT]->SetMaxForce ( 0, wheel_torque );
#endif
  }

  void MaskorEV3Bobb3ePlugin::publishWheelJointState()
  {
    ros::Time current_time = ros::Time::now();

    joint_state_.header.stamp = current_time;
    joint_state_.name.resize ( joints_.size() );
    joint_state_.position.resize ( joints_.size() );

    for ( int i = 0; i < NUM_JOINTS; i++ ) {
      physics::JointPtr joint = joints_[i];
      math::Angle angle = joint->GetAngle ( 0 );
      joint_state_.name[i] = joint->GetName();
      joint_state_.position[i] = angle.Radian () ;
    }
    joint_state_publisher_.publish ( joint_state_ );
  }

  void MaskorEV3Bobb3ePlugin::publishWheelTF()
  {
    ros::Time current_time = ros::Time::now();
    // Todo: auf 3 ändern
    for ( int i = 0; i < NUM_JOINTS; i++ ) {

      std::string wheel_frame = gazebo_ros_->resolveTF(joints_[i]->GetChild()->GetName ());
      std::string wheel_parent_frame = gazebo_ros_->resolveTF(joints_[i]->GetParent()->GetName ());

      math::Pose poseWheel = joints_[i]->GetChild()->GetRelativePose();

      tf::Quaternion qt ( poseWheel.rot.x, poseWheel.rot.y, poseWheel.rot.z, poseWheel.rot.w );
      tf::Vector3 vt ( poseWheel.pos.x, poseWheel.pos.y, poseWheel.pos.z );

      tf::Transform tfWheel ( qt, vt );
      transform_broadcaster_->sendTransform (tf::StampedTransform ( tfWheel, current_time, wheel_parent_frame, wheel_frame ) );
    }
  }

  // Update the controller
  void MaskorEV3Bobb3ePlugin::UpdateChild()
  {
    /* force reset SetParam("fmax") since Joint::Reset reset MaxForce to zero at
       https://bitbucket.org/osrf/gazebo/src/8091da8b3c529a362f39b042095e12c94656a5d1/gazebo/physics/Joint.cc?at=gazebo2_2.2.5#cl-331
       (this has been solved in https://bitbucket.org/osrf/gazebo/diff/gazebo/physics/Joint.cc?diff2=b64ff1b7b6ff&at=issue_964 )
       and Joint::Reset is called after ModelPlugin::Reset, so we need to set maxForce to wheel_torque other than MaskorEV3Bobb3ePlugin::Reset
       (this seems to be solved in https://bitbucket.org/osrf/gazebo/commits/ec8801d8683160eccae22c74bf865d59fac81f1e)
    */
    for ( int i = 0; i < 3; i++ ) {
#if GAZEBO_MAJOR_VERSION > 2
      if ( fabs(wheel_torque -joints_[i]->GetParam ( "fmax", 0 )) > 1e-6 ) {
        joints_[i]->SetParam ( "fmax", 0, wheel_torque );
#else
	if ( fabs(wheel_torque -joints_[i]->GetMaxForce ( 0 )) > 1e-6 ) {
	  joints_[i]->SetMaxForce ( 0, wheel_torque);
#endif
	}
      }

      if ( odom_source_ == ENCODER ) UpdateOdometryEncoder();
      common::Time current_time = parent->GetWorld()->GetSimTime();
      double seconds_since_last_update = ( current_time - last_update_time_ ).Double();

      if ( seconds_since_last_update > update_period_ ) {
        if (this->publish_odom_tf_) publishOdometry ( seconds_since_last_update );
        if ( publishWheelTF_ ) publishWheelTF();
        if ( publishWheelJointState_ ) publishWheelJointState();

        // Update robot in case new velocities have been requested
        getWheelVelocities();

        double current_speed[NUM_JOINTS]; //!

        current_speed[RIGHT_FRONT_WHEEL] = joints_[RIGHT_FRONT_WHEEL]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
	current_speed[RIGHT_REAR_WHEEL] = joints_[RIGHT_REAR_WHEEL]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
	current_speed[LEFT_FRONT_WHEEL] = joints_[LEFT_FRONT_WHEEL]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );
        current_speed[LEFT_REAR_WHEEL] = joints_[RIGHT_FRONT_WHEEL]->GetVelocity ( 0 )   * ( wheel_diameter_ / 2.0 );


        if ( wheel_accel == 0 ||
	     ( fabs ( joint_speeds_[LEFT_FRONT_WHEEL] - current_speed[LEFT_FRONT_WHEEL] ) < 0.01 ) ||
	     ( fabs ( joint_speeds_[RIGHT_FRONT_WHEEL] - current_speed[RIGHT_FRONT_WHEEL] ) < 0.01 ) ) {
	  //if max_accel == 0, or target speed is reached
#if GAZEBO_MAJOR_VERSION > 2
	  joints_[RIGHT_FRONT_WHEEL]->SetParam ( "vel", 0, joint_speeds_[RIGHT_FRONT_WHEEL]/ ( wheel_diameter_ / 2.0 ) );
	  joints_[RIGHT_FRONT_WHEEL]->SetParam ( "vel", 0, joint_speeds_[RIGHT_FRONT_WHEEL]/ ( wheel_diameter_ / 2.0 ) );
#else
	  joints_[RIGHT_FRONT_WHEEL]->SetVelocity ( 0, joint_speeds_[RIGHT_FRONT_WHEEL]/ ( wheel_diameter_ / 2.0 ) );
	  joints_[RIGHT_REAR_WHEEL]->SetVelocity ( 0, joint_speeds_[RIGHT_REAR_WHEEL]/ ( wheel_diameter_ / 2.0 ) );
	  joints_[LEFT_FRONT_WHEEL]->SetVelocity ( 0, joint_speeds_[LEFT_FRONT_WHEEL]/ ( wheel_diameter_ / 2.0 ) );
	  joints_[LEFT_REAR_WHEEL]->SetVelocity ( 0, joint_speeds_[LEFT_REAR_WHEEL]/ ( wheel_diameter_ / 2.0 ) );

	  //joints_[LIFT]->SetVelocity ( 0, joint_speeds_[LIFT] );
#endif
        } else {
	  if ( joint_speeds_[LEFT_FRONT_WHEEL]>=current_speed[LEFT_FRONT_WHEEL] ){
	    joint_speeds_instr_[LEFT_FRONT_WHEEL]+=fmin ( joint_speeds_[LEFT_FRONT_WHEEL]-current_speed[LEFT_FRONT_WHEEL],  wheel_accel * seconds_since_last_update );
	    joint_speeds_instr_[LEFT_REAR_WHEEL]+=fmin ( joint_speeds_[LEFT_REAR_WHEEL]-current_speed[LEFT_REAR_WHEEL],  wheel_accel * seconds_since_last_update );
	  }
	  else {
	    joint_speeds_instr_[LEFT_FRONT_WHEEL]+=fmax ( joint_speeds_[LEFT_FRONT_WHEEL]-current_speed[LEFT_FRONT_WHEEL], -wheel_accel * seconds_since_last_update );
	    joint_speeds_instr_[LEFT_REAR_WHEEL]+=fmax ( joint_speeds_[LEFT_REAR_WHEEL]-current_speed[LEFT_REAR_WHEEL], -wheel_accel * seconds_since_last_update );
	  }

	  if ( joint_speeds_[RIGHT_FRONT_WHEEL]>current_speed[RIGHT_FRONT_WHEEL] ) {
	    joint_speeds_instr_[RIGHT_FRONT_WHEEL]+=fmin ( joint_speeds_[RIGHT_FRONT_WHEEL]-current_speed[RIGHT_FRONT_WHEEL], wheel_accel * seconds_since_last_update );
	    joint_speeds_instr_[RIGHT_REAR_WHEEL]+=fmin ( joint_speeds_[RIGHT_REAR_WHEEL]-current_speed[RIGHT_REAR_WHEEL], wheel_accel * seconds_since_last_update );
	  }
	  else {
	    joint_speeds_instr_[RIGHT_FRONT_WHEEL]+=fmax ( joint_speeds_[RIGHT_FRONT_WHEEL]-current_speed[RIGHT_FRONT_WHEEL], -wheel_accel * seconds_since_last_update );
	    joint_speeds_instr_[RIGHT_REAR_WHEEL]+=fmax ( joint_speeds_[RIGHT_REAR_WHEEL]-current_speed[RIGHT_REAR_WHEEL], -wheel_accel * seconds_since_last_update );
	    //geändert
	    //joint_speeds_instr_[LIFT]+=fmax ( joint_speeds_[LIFT]-current_speed[LIFT], wheel_accel * seconds_since_last_update );
            // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[LEFT], joint_speeds_[LEFT]);
            // ROS_INFO_NAMED("diff_drive", "actual wheel speed = %lf, issued wheel speed= %lf", current_speed[RIGHT],joint_speeds_[RIGHT]);
	  }

#if GAZEBO_MAJOR_VERSION > 2
	  joints_[RIGHT_FRONT_WHEEL]->SetParam ( "vel", 0, joint_speeds_instr_[RIGHT_FRONT_WHEEL] / ( wheel_diameter_ / 2.0 ) );
	  joints_[RIGHT_REAR_WHEEL]->SetParam ( "vel", 0, joint_speeds_instr_[RIGHT_REAR_WHEEL] / ( wheel_diameter_ / 2.0 ) );
	  joints_[LEFT_FRONT_WHEEL]->SetParam ( "vel", 0, joint_speeds_instr_[LEFT_FRONT_WHEEL] / ( wheel_diameter_ / 2.0 ) );
	  joints_[LEFT_REAR_WHEEL]->SetParam ( "vel", 0, joint_speeds_instr_[LEFT_REAR_WHEEL] / ( wheel_diameter_ / 2.0 ) );
	  joints_[LEFT_ARM]->SetParam ( "vel", 0, 0.1 );
	  joints_[RIGHT_ARM]->SetParam ( "vel", 0, 0.2 );
	  joints_[FORK_LIFT]->SetParam ( "vel", 0, 0.1 );
#else
	  joints_[RIGHT_FRONT_WHEEL]->SetVelocity ( 0,joint_speeds_instr_[RIGHT_FRONT_WHEEL] / ( wheel_diameter_ / 2.0 ) );
	  joints_[RIGHT_REAR_WHEEL]->SetVelocity ( 0,joint_speeds_instr_[RIGHT_REAR_WHEEL] / ( wheel_diameter_ / 2.0 ) );
	  joints_[LEFT_FRONT_WHEEL]->SetVelocity ( 0,joint_speeds_instr_[LEFT_FRONT_WHEEL] / ( wheel_diameter_ / 2.0 ) );
	  joints_[LEFT_REAR_WHEEL]->SetVelocity ( 0,joint_speeds_instr_[LEFT_REAR_WHEEL] / ( wheel_diameter_ / 2.0 ) );
	  joints_[LEFT_ARM]->SetVelocity ( 0, -2 );
	  joints_[RIGHT_ARM]->SetVelocity ( 0, 0.01 );
	  joints_[FORK_LIFT]->SetVelocity ( 0, 0.01 );
	  // joints_[LIFT]->SetVelocity ( 0,joint_speeds_instr_[LIFT]);
#endif
        }
        last_update_time_+= common::Time ( update_period_ );
      }
    }

    // Finalize the controller
    void MaskorEV3Bobb3ePlugin::FiniChild()
    {
      alive_ = false;
      queue_.clear();
      queue_.disable();
      gazebo_ros_->node()->shutdown();
      callback_queue_thread_.join();
    }

    void MaskorEV3Bobb3ePlugin::getWheelVelocities()
    {
      boost::mutex::scoped_lock scoped_lock ( lock );

      double vr = x_;
      double lr = z_;
      double va = rot_;

      if(legacy_mode_)
	{
	  joint_speeds_[RIGHT_FRONT_WHEEL] = vr + va * wheel_separation_ / 2.0;
	  joint_speeds_[RIGHT_REAR_WHEEL] = vr + va * wheel_separation_ / 2.0;
	  joint_speeds_[LEFT_FRONT_WHEEL] = vr - va * wheel_separation_ / 2.0;
	  joint_speeds_[LEFT_REAR_WHEEL] = vr - va * wheel_separation_ / 2.0;
	  //joint_speeds_[LIFT] = lr;
	}
      else
	{
	  joint_speeds_[RIGHT_FRONT_WHEEL] = vr + va * wheel_separation_ / 2.0;
	  joint_speeds_[RIGHT_REAR_WHEEL] = vr + va * wheel_separation_ / 2.0;
	  joint_speeds_[LEFT_FRONT_WHEEL] = vr - va * wheel_separation_ / 2.0;
	  joint_speeds_[LEFT_REAR_WHEEL] = vr - va * wheel_separation_ / 2.0;
	  //joint_speeds_[LIFT] = lr;
	}
    }

    void MaskorEV3Bobb3ePlugin::cmdVelCallback ( const geometry_msgs::Twist::ConstPtr& cmd_msg )
    {
      boost::mutex::scoped_lock scoped_lock ( lock );
      x_ = cmd_msg->linear.x;
      z_ = cmd_msg->linear.z;
      rot_ = cmd_msg->angular.z;
    }

    void MaskorEV3Bobb3ePlugin::QueueThread()
    {
      static const double timeout = 0.01;

      while ( alive_ && gazebo_ros_->node()->ok() ) {
        queue_.callAvailable ( ros::WallDuration ( timeout ) );
      }
    }

    void MaskorEV3Bobb3ePlugin::UpdateOdometryEncoder()
    {
      double vl = (joints_[LEFT_FRONT_WHEEL]->GetVelocity ( 0 ) +  joints_[LEFT_REAR_WHEEL]->GetVelocity ( 0 ) ) / 2.0 ;
      double vr = (joints_[RIGHT_FRONT_WHEEL]->GetVelocity ( 0 ) +  joints_[RIGHT_REAR_WHEEL]->GetVelocity ( 0 ) ) / 2.0 ;

      common::Time current_time = parent->GetWorld()->GetSimTime();
      double seconds_since_last_update = ( current_time - last_odom_update_ ).Double();
      last_odom_update_ = current_time;

      double b = wheel_separation_;

      // Book: Sigwart 2011 Autonompus Mobile Robots page:337
      double sl = vl * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
      double sr = vr * ( wheel_diameter_ / 2.0 ) * seconds_since_last_update;
      double ssum = sl + sr;

      double sdiff;
      if(legacy_mode_)
	{
	  sdiff = sl - sr;
	}
      else
	{

	  sdiff = sr - sl;
	}

      double dx = ( ssum ) /2.0 * cos ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
      double dy = ( ssum ) /2.0 * sin ( pose_encoder_.theta + ( sdiff ) / ( 2.0*b ) );
      double dtheta = ( sdiff ) /b;

      pose_encoder_.x += dx;
      pose_encoder_.y += dy;
      pose_encoder_.theta += dtheta;

      double w = dtheta/seconds_since_last_update;
      double v = sqrt ( dx*dx+dy*dy ) /seconds_since_last_update;

      tf::Quaternion qt;
      tf::Vector3 vt;
      qt.setRPY ( 0,0,pose_encoder_.theta );
      vt = tf::Vector3 ( pose_encoder_.x, pose_encoder_.y, 0 );

      odom_.pose.pose.position.x = vt.x();
      odom_.pose.pose.position.y = vt.y();
      odom_.pose.pose.position.z = vt.z();

      odom_.pose.pose.orientation.x = qt.x();
      odom_.pose.pose.orientation.y = qt.y();
      odom_.pose.pose.orientation.z = qt.z();
      odom_.pose.pose.orientation.w = qt.w();

      odom_.twist.twist.angular.z = w;
      odom_.twist.twist.linear.x = dx/seconds_since_last_update;
      odom_.twist.twist.linear.y = dy/seconds_since_last_update;
    }

    void MaskorEV3Bobb3ePlugin::publishOdometry ( double step_time )
    {

      ros::Time current_time = ros::Time::now();
      std::string odom_frame = gazebo_ros_->resolveTF ( odometry_frame_ );
      std::string base_footprint_frame = gazebo_ros_->resolveTF ( robot_base_frame_ );

      tf::Quaternion qt;
      tf::Vector3 vt;

      if ( odom_source_ == ENCODER ) {
        // getting data form encoder integration
        qt = tf::Quaternion ( odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z, odom_.pose.pose.orientation.w );
        vt = tf::Vector3 ( odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z );

      }
      if ( odom_source_ == WORLD ) {
        // getting data form gazebo world
        math::Pose pose = parent->GetWorldPose();
        qt = tf::Quaternion ( pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w );
        vt = tf::Vector3 ( pose.pos.x, pose.pos.y, pose.pos.z );

        odom_.pose.pose.position.x = vt.x();
        odom_.pose.pose.position.y = vt.y();
        odom_.pose.pose.position.z = vt.z();

        odom_.pose.pose.orientation.x = qt.x();
        odom_.pose.pose.orientation.y = qt.y();
        odom_.pose.pose.orientation.z = qt.z();
        odom_.pose.pose.orientation.w = qt.w();

        // get velocity in /odom frame
        math::Vector3 linear;
        linear = parent->GetWorldLinearVel();
        odom_.twist.twist.angular.z = parent->GetWorldAngularVel().z;

        // convert velocity to child_frame_id (aka base_footprint)
        float yaw = pose.rot.GetYaw();
        odom_.twist.twist.linear.x = cosf ( yaw ) * linear.x + sinf ( yaw ) * linear.y;
        odom_.twist.twist.linear.y = cosf ( yaw ) * linear.y - sinf ( yaw ) * linear.x;
      }

      tf::Transform base_footprint_to_odom ( qt, vt );
      transform_broadcaster_->sendTransform (
					     tf::StampedTransform ( base_footprint_to_odom, current_time,
								    odom_frame, base_footprint_frame ) );


      // set covariance
      odom_.pose.covariance[0] = 0.00001;
      odom_.pose.covariance[7] = 0.00001;
      odom_.pose.covariance[14] = 1000000000000.0;
      odom_.pose.covariance[21] = 1000000000000.0;
      odom_.pose.covariance[28] = 1000000000000.0;
      odom_.pose.covariance[35] = 0.001;


      // set header
      odom_.header.stamp = current_time;
      odom_.header.frame_id = odom_frame;
      odom_.child_frame_id = base_footprint_frame;

      odometry_publisher_.publish ( odom_ );
    }

    GZ_REGISTER_MODEL_PLUGIN ( MaskorEV3Bobb3ePlugin )
      }
