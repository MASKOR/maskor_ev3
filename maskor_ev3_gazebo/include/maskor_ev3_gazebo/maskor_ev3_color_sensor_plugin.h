#ifndef MASKOR_EV3_COLOR_SENSOR_HH
#define MASKOR_EV3_COLOR_SENSOR_HH

#include <string>

// library for processing camera data for gazebo / ros conversions
#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>
//OPENCV stuff
#include <opencv2/imgproc/imgproc.hpp>

namespace gazebo
{
  class MaskorEV3ColorSensor : public CameraPlugin, GazeboRosCameraUtils
  {

    public:
      MaskorEV3ColorSensor();
      ~MaskorEV3ColorSensor();

      void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

      void color_detection(int * rgb, cv::Mat H_histo,cv::Mat S_histo, cv::Mat V_histo, bool * color);

    /// Callback for Camera Sensor
    protected:
      virtual void OnNewFrame(const unsigned char *_image,
      unsigned int _width, unsigned int _height,
      unsigned int _depth, const std::string &_format);

    private:
      ros::NodeHandle _nh;
      ros::Publisher _sensorPublisher;

      int LowerH = 0;
      int LowerS = 0;
      int LowerV = 0;
      int UpperH = 180;
      int UpperS = 196;
      int UpperV = 170;
      int count = 0;


  };
}
#endif
