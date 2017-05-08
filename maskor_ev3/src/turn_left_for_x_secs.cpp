#include "ros.h"
#include "ros/time.h"
#include "maskor_ev3/turn_left_for_x_secs.h"
#include "geometry_msgs/Twist.g"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "turn_left_pub");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<geometry_msgs::Twist>("turn_left", 1000);
  ros::Rate loop_rate(10);
  
  ROS_INFO("Ready to turn!");

  while(ros::ok())
    {
      geometry_msgs::Twist msg;
      msg.linear.x=10;
      msg.angular.z=10;
      pub.publish(msg);
      rate.sleep();
    }
  return 0;
}
