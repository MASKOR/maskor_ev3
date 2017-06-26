#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <maskor_ev3_msgs/ColorSensor.h>

double speed = 0;

geometry_msgs::Twist cmd_vel_msg;


void colourCallback(const maskor_ev3_msgs::ColorSensor::ConstPtr& msg)
{
  ROS_INFO("I received a colour scan message");

}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "colour_scan_sub");
  ros::NodeHandle n;
  
  ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  ros::Subscriber colour_scan_sub = n.subscribe("/bobb3e/color_sensor", 1000, colourCallback);
  
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
