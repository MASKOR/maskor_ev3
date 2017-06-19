/*
 * MASKOR EV3 COLOR SENSOR PLUGIN
 *
 * Copyright (c) 2017 
 * Marcel Stüttgen 
 * stuettgen@fh-aachen.de
 * Dennis Miltz 
 * dennis.miltz@alumni.fh-aachen.de
 * https://www.maskor.fh-aachen.de
 *
*/

#include <maskor_ev3_gazebo/maskor_ev3_color_sensor_plugin.h>

namespace gazebo 
{

  MaskorEV3ColorSensorPlugin::MaskorEV3ColorSensorPlugin() {}

  MaskorEV3ColorSensorPlugin::~MaskorEV3ColorSensorPlugin() {}

  // Load the controller
  void MaskorEV3ColorSensorPlugin::Load(physics::ModelPtr parent, 
      sdf::ElementPtr sdf) 
  {

    gazebo_ros_ = GazeboRosPtr ( new GazeboRos ( parent, sdf, "maskor_ev3_color_sensor_plugin" ) );
    gazebo_ros_->isInitialized();
  
    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("MaskorEV3ColorSensorPlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    color_sensor_topic_ = "bobb3e/color_sensor";
    if (!sdf->HasElement("colorSensorTopic")) 
    {
      ROS_WARN("MaskorEV3ColorSensorPlugin (ns = %s) missing <colorSensorTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), color_sensor_topic_.c_str());
    } 
    else 
    {
      color_sensor_topic_ = sdf->GetElement("colorSensorTopic")->Get<std::string>();
    }

    color_sensor_frame_ = "color_sensor_link";
    if (!sdf->HasElement("colorSensorFrame")) 
    {
      ROS_WARN("MaskorEV3ColorSensorPlugin (ns = %s) missing <colorSensorFrame>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), color_sensor_frame_.c_str());
    } 
    else 
    {
      color_sensor_frame_ = sdf->GetElement("colorSensorFrame")->Get<std::string>();
    }
 
    rate_ = 20.0;
    if (!sdf->HasElement("rate")) 
    {
      ROS_WARN("MaskorEV3ColorSensorPlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), rate_);
    } 
    else 
    {
      rate_ = sdf->GetElement("rate")->Get<double>();
    } 
 
    last_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_pose_ = parent_->GetWorldPose();


    color_ = 0;
    alive_ = true;

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("MaskorEV3ColorSensorPlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("MaskorEV3ColorSensorPlugin (%s) has started!", 
        robot_namespace_.c_str());

  //tf_prefix_ = tf::getPrefixParam(*rosnode_);

 color_sensor_pub_ = rosnode_->advertise<maskor_ev3_msgs::ColorSensor>(color_sensor_topic_, 1);

    it = new image_transport::ImageTransport(*rosnode_);
        
    //image_sub = it->subscribe("/bobb3e/camera1/image_raw", 1, boost::bind(&MaskorEV3ColorSensorPlugin::colorDetectionCallback, this, _1));
    
      
    // start custom queue 
    callback_queue_thread_ = 
      boost::thread(boost::bind(&MaskorEV3ColorSensorPlugin::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MaskorEV3ColorSensorPlugin::UpdateChild, this));

  }

  // Update the controller
  void MaskorEV3ColorSensorPlugin::UpdateChild() 
  {
    boost::mutex::scoped_lock scoped_lock(lock);

    common::Time current_time = parent_->GetWorld()->GetSimTime();
    double seconds_since_last_update = (current_time - last_publish_time_).Double();
    
    //TODO : Update internal variables
    

    if (rate_ > 0.0) {
      if (seconds_since_last_update > (1.0 / rate_)) {
        publishColorSensorMessage();
        last_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void MaskorEV3ColorSensorPlugin::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

 
  void MaskorEV3ColorSensorPlugin::QueueThread() 
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void MaskorEV3ColorSensorPlugin::publishColorSensorMessage()
  {
    /*
      # Value Color
      # 0 	none
      # 1 	black
      # 2 	blue
      # 3 	green
      # 4 	yellow
      # 5 	red
      # 6 	white
      # 7 	brown
    */

    color_sensor_msg_.header.stamp = ros::Time::now();
    color_sensor_msg_.header.frame_id = color_sensor_frame_;
    color_sensor_msg_.color = color_;
    
    color_sensor_pub_.publish(color_sensor_msg_);
  }


  
  void colorDetectionCallback(const sensor_msgs::ImageConstPtr& original_image) {
    ROS_INFO("colorDetectionCallback(const sensor_msgs::ImageConstPtr& original_image");

  }
    

  
  GZ_REGISTER_MODEL_PLUGIN(MaskorEV3ColorSensorPlugin)
}
