#include "ros.h"
#include "maskor_ev3/turn_left_for_x_secs.h"
#include "ros/time.h"

void turn_left(maskor_ev3::

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turn_left_server");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise("turn_left", 1000);
  ros::Rate loop_rate(10);
  
  ROS_INFO("Ready to turn!");
  ros::spin();

  return 0;
}
