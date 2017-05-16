//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/Int8.h>



//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "Image Processed";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
ros::Publisher chatter_pub;

int LowerH = 0;
int LowerS = 0;
int LowerV = 0;
int UpperH = 180;
int UpperS = 196;
int UpperV = 170;
int count = 0;




void color_detection(int * rgb, cv::Mat H_histo,cv::Mat S_histo, cv::Mat V_histo, bool * color) {

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
    else if(rgb[0] >= 240 && rgb[0] <= 250 && rgb[1] >= 250 && rgb[1] <= 255 && rgb[2] >= 53 && rgb[2] <= 63){
      color[5] = true; // yellow = true
    }
    else if(rgb[0] >= 250 && rgb[0] <= 255 && rgb[1] >= 0 && rgb[1] <= 10 && rgb[2] >= 0 && rgb[2] <= 10){
      color[2] = true; // red = true
    }
    else if(rgb[0] >= 98 && rgb[0] <= 107 && rgb[1] >= 46 && rgb[1] <= 55 && rgb[2] >= 49 && rgb[2] <= 59){
      color[6] = true; // brown = true
    }
    else{
       color[7] = true; // undifined = true;
    }
  }

}

void colorDetectionCallback(const sensor_msgs::ImageConstPtr& original_image)
{


    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        //std::cout << "Hey" << std::endl;
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
  cv::Mat img_mask,img_hsv, img_hsv_1;
  cv::cvtColor(cv_ptr->image,img_hsv,CV_BGR2HSV);
  img_hsv_1 = img_hsv;
  cv::inRange(img_hsv,cv::Scalar(LowerH,LowerS,LowerV),cv::Scalar(UpperH,UpperS,UpperV),img_mask);
    //Display the image using OpenCV
    cv::imshow(WINDOW, img_mask);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.

// Farberkennung RGB
    int rgb[3] = {0,0,0};

    for(int a=0; a<(cv_ptr->image.rows * cv_ptr->image.cols *3); a++){
      //ROS_INFO("Image_Data[%d] %d",a,cv_ptr->image.data[a]);
      if(count > 2){
        count = 0;
      }
      rgb[count] += cv_ptr->image.data[a];
      count++;
    }

    for(int i = 0;i<3;i++){
      rgb[i] = rgb[i] / 1600;
    }

/*    if(rgb[0] >=0 && rgb[0] <= 10 && rgb[1] >=0 && rgb[1] <= 10 && rgb[2] >=0 && rgb[2] <= 20){
      ROS_INFO("Schwarz");
    }
    else if(rgb[0] >=0 && rgb[0] <= 10 && rgb[1] >=0 && rgb[1] <= 10 && rgb[2] >=250 && rgb[2] <= 255){
      ROS_INFO("Blau");
    }
    else if(rgb[0] >= 0 && rgb[0] <= 10 && rgb[1] >= 250 && rgb[1] <= 255 && rgb[2] >=55 && rgb[2] <= 62){
      ROS_INFO("Grün");
    }
    else if(rgb[0] >= 240 && rgb[0] <= 250 && rgb[1] >= 250 && rgb[1] <= 255 && rgb[2] >= 53 && rgb[2] <= 63){
      ROS_INFO("Gelb");
    }
    else if(rgb[0] >= 250 && rgb[0] <= 255 && rgb[1] >= 0 && rgb[1] <= 10 && rgb[2] >= 0 && rgb[2] <= 10){
      ROS_INFO("Rot");
    }
    else if(rgb[0] >= 245 && rgb[0] <= 255 && rgb[1] >= 245 && rgb[1] <= 255 && rgb[2] >= 245 && rgb[2] <= 255){
      ROS_INFO("Weiß");
    }
    else if(rgb[0] >= 98 && rgb[0] <= 107 && rgb[1] >= 46 && rgb[1] <= 55 && rgb[2] >= 49 && rgb[2] <= 59){
      ROS_INFO("Braun");
    }
    else{
      ROS_INFO("Unbekannte Farbe oder Farblos");
    }

//    ROS_INFO("rgb[0] %d",rgb[0]);
//    ROS_INFO("rgb[1] %d",rgb[1]);
//    ROS_INFO("rgb[2] %d",rgb[2]); */

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

        /// Display
        cv::namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
        imshow("calcHist Demo", histImage );


        //Color Detection for black and white
      color_detection(rgb,H_hist, S_hist, V_hist, color);

      std_msgs::Int8 msg;

     if(color[0] == true){
        msg.data = 0;
        std::cout << "Color: black" << std::endl;
      }
      else if(color[1] == true){
        msg.data = 1;
        std::cout << "Color: white" << std::endl;
      }
     else if(color[2] ==true){
       msg.data = 2;
       std::cout << "Color: red" << std::endl;
     }
     else if(color[3] ==true){
       msg.data = 3;
       std::cout << "Color: green" << std::endl;
     }
     else if(color[4] ==true){
       msg.data = 4;
       std::cout << "Color: blue" << std::endl;
     }
     else if(color[5] ==true){
       msg.data = 5;
       std::cout << "Color: yellow" << std::endl;
     }
     else if(color[6] ==true){
       msg.data = 6;
       std::cout << "Color: brown" << std::endl;
     }
     else if(color[7] ==true){
       msg.data = 7;
       std::cout << "Color: undifined" << std::endl;
     }



     ROS_INFO("%d", msg.data);
     chatter_pub.publish(msg);


//      std::cout << "H_hist" << H_hist << std::endl;
//      std::cout << "S_hist" << S_hist << std::endl;
//      std::cout << "V_hist" << V_hist << std::endl;

//    std::cout << HSV[0] / 1600 << std::endl;
//    std::cout << HSV[1] / 1600 << std::endl;
//    std::cout << HSV[2] / 1600 << std::endl;



    pub.publish(cv_ptr->toImageMsg());
}
//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{

    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }



    //Invert Image
    //Go through all the rows
    for(int i=0; i<cv_ptr->image.rows; i++)
    {
        //Go through all the columns
        for(int j=0; j<cv_ptr->image.cols; j++)
        {
            //Go through all the channels (b, g, r)
            for(int k=0; k<cv_ptr->image.channels(); k++)
            {
                //Invert the image by subtracting image data from 255
                cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k] = 255-cv_ptr->image.data[i*cv_ptr->image.rows*4+j*3 + k];
            }
        }
    }


    //Display the image using OpenCV
    cv::imshow(WINDOW, cv_ptr->image);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
        pub.publish(cv_ptr->toImageMsg());
}

/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
    std::cout <<"startin color_sonsor_node"<< std::endl;
    ROS_INFO("startin color_sonsor_node");
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
    * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
    * The name used here must be a base name, ie. it cannot have a / in it.
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "color_sensor_node");
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;

    chatter_pub = nh.advertise<std_msgs::Int8>("chatter", 1000);
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

  cv::namedWindow("Ball");
  cv::createTrackbar("LowerH","Ball",&LowerH,180,NULL);
  cv::createTrackbar("UpperH","Ball",&UpperH,180,NULL);
  cv::createTrackbar("LowerS","Ball",&LowerS,256,NULL);
  cv::createTrackbar("UpperS","Ball",&UpperS,256,NULL);
  cv::createTrackbar("LowerV","Ball",&LowerV,256,NULL);
  cv::createTrackbar("UpperV","Ball",&UpperV,256,NULL);

    //OpenCV HighGUI call to create a display window on start-up.
    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    /**
    * Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used.
    * In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call
    * the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
    * subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe.
    * When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
    */
    //image_transport::Subscriber sub = it.subscribe("camera/image_raw", 1, imageCallback);
    image_transport::Subscriber sub = it.subscribe("/rrbot/camera1/image_raw", 1, colorDetectionCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    cv::destroyWindow(WINDOW);
    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    pub = it.advertise("camera/image_processed", 1);
    /**
    * In this application all user callbacks will be called from within the ros::spin() call.
    * ros::spin() will not return until the node has been shutdown, either through a call
    * to ros::shutdown() or a Ctrl-C.
    */
    ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");

}
