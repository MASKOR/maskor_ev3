#include <ros/ros.h>
#include "gazebo_plugins/gazebo_ros_camera.h"

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/common/Plugin.hh>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <maskor_ev3_gazebo/maskor_ev3_color_sensor_plugin.h>
#include <maskor_ev3_msgs/ColorSensor.h>

#include <string>


namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(MaskorEV3ColorSensor)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  MaskorEV3ColorSensor::MaskorEV3ColorSensor():
  _nh("MaskorEV3ColorSensor"),
    LowerH(0), LowerS(0), LowerV(0), UpperH(180), UpperS(196), UpperV(170),
    count(0)
  {
    //_sensorPublisher = _nh.advertise<std_msgs::Int8>("/maskor_ev3_color_sensor", 1);
    _sensorPublisher = _nh.advertise<maskor_ev3_msgs::ColorSensor>("/bobb3e/color_sensor", 1);
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
		cv::Mat cv_camFrame = cv::Mat(_height, _width,  CV_8UC3, data);
		//converting from gazebo RGB to openCV BGR format
		cv::cvtColor(cv_camFrame, cv_camFrame, cv::COLOR_RGB2BGR);


		if(_showDebugWindows) {
		  //show Image just for debug
		  imshow("OPENCV DEBUG OUTPUT",cv_camFrame);
		  cv::waitKey(1);
		}
		/////////////////////// CALCULATE DETECTED COLOR ///////////////////////////////

		cv::Mat img_mask,img_hsv, img_hsv_1;
		cv::cvtColor(cv_camFrame,img_hsv,CV_BGR2HSV);
		img_hsv_1 = img_hsv;
		cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask);

		if(_showDebugWindows) {
		  //Display the image using OpenCV

		  cv::imshow("Image Processed", img_mask);
		  //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
		  cv::waitKey(3);
		}


		// Farberkennung RGB
		int rgb[3] = {0,0,0};

		for(int a=0; a<(cv_camFrame.rows * cv_camFrame.cols *3); a++){
		  //ROS_INFO("Image_Data[%d] %d",a,cv_ptr->image.data[a]);
		  if(count > 2){
		    count = 0;
		  }
		  rgb[count] += cv_camFrame.data[a];
		  count++;
		}

		for(int i = 0;i<3;i++){
		  rgb[i] = rgb[i] / 1600;
		}

		//Farberkennung HSV
		//color[0] = black
		//color[1] = white
		//color[2] = red
		//color[3] = green
		//color[4] = blue
		//color[5] = yellow
		//color[6] = brown
		//color[7] = undifined
		bool color[8] = {false};

		cv::Mat hsv[3];   //HSV array
		split(img_hsv_1, hsv);//split source

		int HSV[3] = {0,0,0};


		for(int a=0; a<3; a++){
		  for(int c = 0;c< hsv[a].cols;c++){
		    for(int r = 0;r<hsv[a].rows;r++){
		      HSV[a] += (int)hsv[a].at<uchar>(c,r);
		    }
		  }
		}

		/// Establish the number of bins
		int histSize = 256;

		/// Set the ranges ( for H,S,V) )
		float range[] = { 0, 256 } ;
		const float* histRange = { range };

		bool uniform = true; bool accumulate = false;

		cv::Mat H_hist, S_hist, V_hist;

		/// Compute the histograms:
		cv::calcHist( &hsv[0], 1, 0, cv::Mat(), H_hist, 1, &histSize, &histRange, uniform, accumulate );
		cv::calcHist( &hsv[1], 1, 0, cv::Mat(), S_hist, 1, &histSize, &histRange, uniform, accumulate );
		cv::calcHist( &hsv[2], 1, 0, cv::Mat(), V_hist, 1, &histSize, &histRange, uniform, accumulate );

		//Visualisation
		int hist_w = 512; int hist_h = 400;
		int bin_w = cvRound( (double) hist_w/histSize );

		cv::Mat histImage( hist_h, hist_w, CV_8UC3, cv::Scalar( 0,0,0) );

		cv::Mat H_hist1, S_hist1, V_hist1;

		normalize(H_hist, H_hist1, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
		normalize(S_hist, S_hist1, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );
		normalize(V_hist, V_hist1, 0, histImage.rows, cv::NORM_MINMAX, -1, cv::Mat() );

		/// Draw for each channel
		for( int i = 1; i < histSize; i++ )
		  {
		    line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(H_hist1.at<float>(i-1)) ) ,
			  cv::Point( bin_w*(i), hist_h - cvRound(H_hist1.at<float>(i)) ),
			  cv::Scalar( 255, 0, 0), 2, 8, 0  );
		    line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(S_hist1.at<float>(i-1)) ) ,
			  cv::Point( bin_w*(i), hist_h - cvRound(S_hist1.at<float>(i)) ),
			  cv::Scalar( 0, 255, 0), 2, 8, 0  );
		    line( histImage, cv::Point( bin_w*(i-1), hist_h - cvRound(V_hist1.at<float>(i-1)) ) ,
			  cv::Point( bin_w*(i), hist_h - cvRound(V_hist1.at<float>(i)) ),
			  cv::Scalar( 0, 0, 255), 2, 8, 0  );
		  }

		if(_showDebugWindows) {
		  /// Display
		  cv::namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
		  imshow("calcHist Demo", histImage );
		}

		//Color Detection for black and white
		color_detection(rgb,H_hist, S_hist, V_hist, color);

		maskor_ev3_msgs::ColorSensor color_sensor_msg;

		if(color[0] == true){
		  color_sensor_msg.color = 0;
		  //std::cout << "Color: black" << std::endl;
		}
		else if(color[1] == true){
		  color_sensor_msg.color = 1;
		  //std::cout << "Color: white" << std::endl;
		}
		else if(color[2] ==true){
		  color_sensor_msg.color = 2;
		  //std::cout << "Color: red" << std::endl;
		}
		else if(color[3] ==true){
		  color_sensor_msg.color = 3;
		  //std::cout << "Color: green" << std::endl;
		}
		else if(color[4] ==true){
		  color_sensor_msg.color = 4;
		  //std::cout << "Color: blue" << std::endl;
		}
		else if(color[5] ==true){
		  color_sensor_msg.color = 5;
		  //std::cout << "Color: yellow" << std::endl;
		}
		else if(color[6] ==true){
		  color_sensor_msg.color = 6;
		  //std::cout << "Color: brown" << std::endl;
		}
		else if(color[7] ==true){
		  color_sensor_msg.color = 7;
		  //std::cout << "Color: undifined" << std::endl;
		}

		color_sensor_msg.header.stamp = ros::Time::now();
		color_sensor_msg.header.frame_id = "color_sensor_link";

		_sensorPublisher.publish(color_sensor_msg);

		//free reserved space!
		delete data;

	      }
	  }
      }
  }


  void MaskorEV3ColorSensor::color_detection(int * rgb, cv::Mat H_histo,cv::Mat S_histo, cv::Mat V_histo, bool * color) {

    int black_or_white = 0;
    int black_int = 0;
    int white_int = 0;
    bool color_b = false;

    for(int i = 250;i < 255;i++){
      if((S_histo.at<float>(i)) < 50){
        black_or_white++;
      }
    }

    if(black_or_white >=3)
      {
	for(int i= 0; i < 6;i++){
	  if(V_histo.at<float>(i) > 15){
	    black_int++;
	  }
	}
	if(black_int >= 4)
	  {
	    color[0] = true; //black = true
	  }
	else{
	  for(int i = 0;i< 3; i++){
	    if(S_histo.at<float>(i) > 15){
	      white_int++;
	    }
	  }
	  if(white_int >=1){
	    color[1] = true; //white = true
	  }
	  else{
	    color_b = true;
	  }

	}
      }
    else{
      color_b = true;
    }

    if(color_b != false){

      if(rgb[0] >=0 && rgb[0] <= 10 && rgb[1] >=0 && rgb[1] <= 10 && rgb[2] >=250 && rgb[2] <= 255){
        color[4] = true; // blue = true
      }
      else if(rgb[0] >= 0 && rgb[0] <= 10 && rgb[1] >= 250 && rgb[1] <= 255 && rgb[2] >=0 && rgb[2] <= 10){
        color[3] = true; // green = true
      }
      else if(rgb[0] >= 240 && rgb[0] <= 255 && rgb[1] >= 250 && rgb[1] <= 255 && rgb[2] >= 0 && rgb[2] <= 10){
        color[5] = true; // yellow = true
      }
      else if(rgb[0] >= 250 && rgb[0] <= 255 && rgb[1] >= 0 && rgb[1] <= 10 && rgb[2] >= 0 && rgb[2] <= 10){
        color[2] = true; // red = true
      }
      else if(rgb[0] >= 110 && rgb[0] <= 150 && rgb[1] >= 46 && rgb[1] <= 80 && rgb[2] >= 0 && rgb[2] <= 40){
        color[6] = true; // brown = true
      }
      else{
	color[7] = true; // undifined = true;
      }
    }

  }

}
