#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <std_msgs/Int8.h>

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
  _nh("MaskorEV3ColorSensor")
  {
    _sensorPublisher = _nh.advertise<std_msgs::Int8>("/maskor_ev3_color_sensor", 1);
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
  // Callback for each Camera Image
  void MaskorEV3ColorSensor::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
  {
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


          // storeage for image data
          unsigned char *data;
          data = new unsigned char[_width * _height * 3];

          // copy const _image to  data
          memcpy(data, _image, _width * _height * 3);

          //converting gazebo camera image to CV Mat
          cv::Mat cv_img = cv::Mat(_height, _width,  CV_8UC3, data);
          //converting from gazebo RGB to openCV BGR format
          cv::cvtColor(cv_img, cv_img, cv::COLOR_RGB2BGR);

          //show Image just for debug
          imshow("OPENCV DEBUG OUTPUT",cv_img);
          cv::waitKey(1);


          /// CALCULATE COLOR HERE


          //create ROS message 
          std_msgs::Int8 msg;
          msg.data = 1;

          //publish ROS message
          _sensorPublisher.publish(msg);

          //free reserved space!
          delete data;

        }
      }
    }
  }
}
