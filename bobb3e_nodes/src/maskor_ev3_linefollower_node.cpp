#include "ros/ros.h"
#include "std_msgs/String.h"
#include <maskor_ev3_msgs/ColorSensor.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

ros::Publisher vel_pub_;
ros::Subscriber color_sensor;

int turn_counter;
bool turn_flag;


void drive(int input){
  geometry_msgs::Twist twist;

  //VORWÄRTS
  if(input == 0){
    std::cout << "vörwärts" << '\n';
    twist.linear.x = 0.15;
    twist.angular.z = 0;
    vel_pub_.publish(twist);
  }
  //RÜCKWÄRTS
  else if(input ==1){
    std::cout << "rückwärts" << '\n';
    //lastblackr = false;
    twist.linear.x = -0.15;
    twist.angular.z = 0;
    vel_pub_.publish(twist);
  }
  //RECHTS
  else if(input ==2){
    std::cout << "rechts" << '\n';
    //lastblackr = false;
    twist.linear.x = 0;
    twist.angular.z = -0.08;
    vel_pub_.publish(twist);
  }
  //LINKS
  else if(input ==3){
    std::cout << "links" << '\n';
    twist.linear.x = 0;
    twist.angular.z = 0.08;
    vel_pub_.publish(twist);
  }
  //STOPP
  else if(input == 4){
    std::cout << "stopp" << '\n';
    twist.linear.x = 0;
    twist.angular.z = 0;
    vel_pub_.publish(twist);
  }
}

void lineFollowerCallback(const maskor_ev3_msgs::ColorSensor::ConstPtr& msg)
{
  ROS_INFO("Color: [%i]", msg->color);

  if(msg->color == 1){
    drive(0);
    turn_counter = 0;
  }
  else{
    if(turn_flag == true)
    {
      turn_counter++;
      drive(2);

      if(turn_counter == 20)
        turn_flag = false;
    }
    else
    {
      turn_counter--;
      drive(3);
      if(turn_counter == -20)
        turn_flag = true;
    }
  }


}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "line_follower");

  ros::NodeHandle n;

  vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  color_sensor =  n.subscribe<maskor_ev3_msgs::ColorSensor>("/bobb3e/color_sensor", 1000, &lineFollowerCallback);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {



    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}
