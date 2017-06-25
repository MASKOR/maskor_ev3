#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "maskor_ev3_gazebo/maskor_ev3_color_sensor_plugin.h"

#include "gazebo_plugins/gazebo_ros_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/Illuminance.h>

#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(MaskorEV3ColorSensor)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  MaskorEV3ColorSensor::MaskorEV3ColorSensor():
  _nh("MaskorEV3ColorSensor"),
  _fov(6),
  _range(10)
  {
    _sensorPublisher = _nh.advertise<sensor_msgs::Illuminance>("MaskorEV3ColorSensor", 1);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  MaskorEV3ColorSensor::~MaskorEV3ColorSensor()
  {
    ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
  }

  void MaskorEV3ColorSensor::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }

    CameraPlugin::Load(_parent, _sdf);
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    GazeboRosCameraUtils::Load(_parent, _sdf);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void MaskorEV3ColorSensor::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
    static int seq=0;

    this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();

    if (!this->parentSensor->IsActive())
    {
      if ((*this->image_connect_count_) > 0)
      // enable only if plugin fully loaded. Increases Gazebo startup
        this->parentSensor->SetActive(true);
    }
    else
    {
      if ((*this->image_connect_count_) > 0)
      {
        common::Time cur_time = this->world_->GetSimTime();
        if (cur_time - this->last_update_time_ >= this->update_period_)
        {
          this->PutCameraData(_image);
          this->PublishCameraInfo();
          this->last_update_time_ = cur_time;


          //unsigned char* b = _image;

          cv::Mat TempMat = cv::Mat(_width, _height, CV_8UC1,*_image);


          imshow("this is a test",TempMat);

          // Publish your own data here




          sensor_msgs::Illuminance msg;
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "";
          msg.header.seq = seq;

          int startingPix = _width * ( (int)(_height/2) - (int)( _fov/2)) - (int)(_fov/2);

          double illum = 0;
          for (int i=0; i<_fov ; ++i)
          {
            int index = startingPix + i*_width;
            for (int j=0; j<_fov ; ++j)
              illum += _image[index+j];
          }

          msg.illuminance = illum/(_fov*_fov);
          msg.variance = 0.0;

          _sensorPublisher.publish(msg);

          seq++;
        }
      }
    }
  }
}
