#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

double speed = 0;

geometry_msgs::Twist cmd_vel_msg;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO("I received a laser scan message");
  double minimum = 1000;
  
  
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "laser_scan_sub");
  ros::NodeHandle n;
  
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Subscriber laser_scan_sub = n.subscribe("/bobb3e/scan", 1000, laserCallback);
  
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
  
    //ROS_INFO("publishing cmd_vel message");

    cmd_vel_msg.linear.x = speed;

    cmd_vel_pub.publish(cmd_vel_msg);
    
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
