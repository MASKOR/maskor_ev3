/*
 * MASKOR EV3 GYRO SENSOR PLUGIN
 *
 * Copyright (c) 2017 
 * Marcel Stüttgen 
 * stuettgen@fh-aachen.de
 * https://www.maskor.fh-aachen.de
 *
*/

#include <maskor_ev3_gazebo/maskor_ev3_gyro_sensor_plugin.h>

namespace gazebo 
{

  MaskorEV3GyroSensorPlugin::MaskorEV3GyroSensorPlugin() {}

  MaskorEV3GyroSensorPlugin::~MaskorEV3GyroSensorPlugin() {}

  // Load the controller
  void MaskorEV3GyroSensorPlugin::Load(physics::ModelPtr parent, 
      sdf::ElementPtr sdf) 
  {

    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( parent, sdf, "maskor_ev3_gyro_sensor_plugin" ) );
    gazebo_ros_->isInitialized();
  
    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("MaskorEV3GyroSensorPlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    gyro_sensor_topic_ = "gyro_sensor";
    if (!sdf->HasElement("gyroSensorTopic")) 
    {
      ROS_WARN("MaskorEV3GyroSensorPlugin (ns = %s) missing <gyroSensorTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), gyro_sensor_topic_.c_str());
    } 
    else 
    {
      gyro_sensor_topic_ = sdf->GetElement("gyroSensorTopic")->Get<std::string>();
    }

    gyro_sensor_frame_ = "gyro_sensor_link";
    if (!sdf->HasElement("gyroSensorFrame")) 
    {
      ROS_WARN("MaskorEV3GyroSensorPlugin (ns = %s) missing <gyroSensorFrame>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), gyro_sensor_frame_.c_str());
    } 
    else 
    {
      gyro_sensor_frame_ = sdf->GetElement("gyroSensorFrame")->Get<std::string>();
    }
 
    rate_ = 20.0;
    if (!sdf->HasElement("rate")) 
    {
      ROS_WARN("MaskorEV3GyroSensorPlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), rate_);
    } 
    else 
    {
      rate_ = sdf->GetElement("rate")->Get<double>();
    } 
 
    last_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_pose_ = parent_->GetWorldPose();
    rotational_speed_ = 0;
    angle_ = 0;
    alive_ = true;

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("MaskorEV3GyroSensorPlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("MaskorEV3GyroSensorPlugin (%s) has started!", 
        robot_namespace_.c_str());

    //tf_prefix_ = tf::getPrefixParam(*rosnode_);
 
    gyro_sensor_pub_ = rosnode_->advertise<maskor_ev3_msgs::GyroSensor>(gyro_sensor_topic_, 1);

    // start custom queue 
    callback_queue_thread_ = 
      boost::thread(boost::bind(&MaskorEV3GyroSensorPlugin::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MaskorEV3GyroSensorPlugin::UpdateChild, this));

  }

  // Update the controller
  void MaskorEV3GyroSensorPlugin::UpdateChild() 
  {
    boost::mutex::scoped_lock scoped_lock(lock);

    common::Time current_time = parent_->GetWorld()->GetSimTime();
    double seconds_since_last_update = (current_time - last_publish_time_).Double();
    
    math::Pose pose = parent_->GetWorldPose();
    float yaw = pose.rot.GetYaw();
    float last_yaw = last_pose_.rot.GetYaw();

    //calc angle in degree and rotational speed in degree per second
    angle_ = yaw * (180.0 / M_PI);
    rotational_speed_ = (angle_ - last_yaw * (180.0 / M_PI)) / seconds_since_last_update;

    if (rate_ > 0.0) {
      if (seconds_since_last_update > (1.0 / rate_)) {
        publishGyroSensorMessage();
        last_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void MaskorEV3GyroSensorPlugin::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

 
  void MaskorEV3GyroSensorPlugin::QueueThread() 
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void MaskorEV3GyroSensorPlugin::publishGyroSensorMessage()
  {

    gyro_sensor_msg_.header.stamp = ros::Time::now();
    gyro_sensor_msg_.header.frame_id = gyro_sensor_frame_;
    gyro_sensor_msg_.angle = angle_;
    gyro_sensor_msg_.rotational_speed = rotational_speed_;
    
    gyro_sensor_pub_.publish(gyro_sensor_msg_);

    last_pose_ = parent_->GetWorldPose();
  }
  
  GZ_REGISTER_MODEL_PLUGIN(MaskorEV3GyroSensorPlugin)
}
