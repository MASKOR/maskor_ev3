/*
 * MASKOR EV3 GYRO SENSOR PLUGIN
 *
 * Copyright (c) 2017 
 * Marcel Stüttgen 
 * stuettgen@fh-aachen.de
 * https://www.maskor.fh-aachen.de
 *
*/


#ifndef MASKOR_EV3_GYRO_SENSOR_PLUGIN_HH
#define MASKOR_EV3_GYRO_SENSOR_PLUGIN_HH

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <map>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <ros/advertise_options.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <maskor_ev3_msgs/GyroSensor.h>

namespace gazebo {

  class MaskorEV3GyroSensorPlugin : public ModelPlugin {

    public: 
      MaskorEV3GyroSensorPlugin();
      ~MaskorEV3GyroSensorPlugin();
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected: 
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      void publishGyroSensorMessage();

      physics::ModelPtr parent_;
      event::ConnectionPtr update_connection_;
      boost::shared_ptr<ros::NodeHandle> rosnode_;
      ros::Publisher gyro_sensor_pub_;
      maskor_ev3_msgs::GyroSensor gyro_sensor_msg_;
      std::string tf_prefix_;

      boost::mutex lock;

      std::string robot_namespace_;
      std::string gyro_sensor_topic_;
      std::string gyro_sensor_frame_;

      // Custom Callback Queue
      ros::CallbackQueue queue_;
      boost::thread callback_queue_thread_;
      void QueueThread();

      bool alive_;
      double rate_;
      int rotational_speed_;
      int angle_;
      common::Time last_publish_time_;
      math::Pose last_pose_;
  };

}

#endif /* end of include guard: MASKOR_EV3_GYRO_SENSOR_PLUGIN_HH */
