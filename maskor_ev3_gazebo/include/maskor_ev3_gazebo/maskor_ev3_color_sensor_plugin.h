/*
 * MASKOR EV3 COLOR SENSOR PLUGIN
 *
 * Copyright (c) 2017 
 * Marcel Stüttgen 
 * stuettgen@fh-aachen.de
 *
 * Dennis Miltz 
 * dennis.miltz@alumni.fh-aachen.de
 *
 * https://www.maskor.fh-aachen.de
 *
*/


#ifndef MASKOR_EV3_COLOR_SENSOR_PLUGIN_HH
#define MASKOR_EV3_COLOR_SENSOR_PLUGIN_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <sdf/sdf.hh>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <maskor_ev3_msgs/ColorSensor.h>

namespace gazebo {

  class MaskorEV3ColorSensorPlugin : public ModelPlugin {

    public: 
      MaskorEV3ColorSensorPlugin();
      ~MaskorEV3ColorSensorPlugin();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void colorDetectionCallback(const sensor_msgs::ImageConstPtr& original_image);
      void publishColorSensorMessage();

      GazeboRosPtr gazebo_ros_;
      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;
      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher color_sensor_pub_;
      image_transport::ImageTransport* it;
      image_transport::Subscriber image_sub;
      
      maskor_ev3_msgs::ColorSensor color_sensor_msg_;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string color_sensor_topic_;
      std::string color_sensor_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      bool alive_;
      double rate_;
      int color_;
      
      int LowerH_ = 0;
      int LowerS_ = 0;
      int LowerV_ = 0;
      int UpperH_ = 180;
      int UpperS_ = 196;
      int UpperV_ = 170;
      
      common::Time last_publish_time_;
      math::Pose last_pose_;
  };

}

#endif /* end of include guard: MASKOR_EV3_COLOR_SENSOR_PLUGIN_HH */
