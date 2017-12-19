#include "ros/ros.h"
#include "std_msgs/String.h"
#include <maskor_ev3_msgs/ColorSensor.h>
#include <geometry_msgs/Twist.h>
#include <sstream>

enum direction {
  FORWARD = 0,
  BACKWARD,
  LEFT,
  RIGHT,
  STOP
};


ros::Publisher vel_pub_;
ros::Subscriber color_sensor;

int turn_counter;
bool turn_flag;

void drive(int input){
  geometry_msgs::Twist twist;

  //VORWÄRTS
  if(input == FORWARD){

    ROS_INFO("FORWARD");

    twist.linear.x = 0.15;
    twist.angular.z = 0;
    vel_pub_.publish(twist);
  }
  //RÜCKWÄRTS
  else if(input == BACKWARD){

    ROS_INFO("BACKWARD");

    twist.linear.x = -0.15;
    twist.angular.z = 0;
    vel_pub_.publish(twist);
  }
  //RECHTS
  else if(input == RIGHT){

    ROS_INFO("RIGHT");

    twist.linear.x = 0;
    twist.angular.z = -0.08;
    vel_pub_.publish(twist);
  }
  //LINKS
  else if(input == LEFT){

    ROS_INFO("LEFT");

    twist.linear.x = 0;
    twist.angular.z = 0.08;
    vel_pub_.publish(twist);
  }
  //STOPP
  else if(input == STOP){

    ROS_INFO("STOP");

    twist.linear.x = 0;
    twist.angular.z = 0;
    vel_pub_.publish(twist);
  }
}



void lineFollowerCallback(const maskor_ev3_msgs::ColorSensor::ConstPtr& msg)
{
  int detected_color = msg->color;
  ROS_INFO("detected color: [%i]", detected_color);


  if(detected_color == 1 || detected_color == 5 || detected_color == 2){
      drive(FORWARD);
      turn_counter = 0;
    }
    else{
      if(turn_flag == true)
      {
        turn_counter++;
        drive(RIGHT);

        if(turn_counter == 20)
          turn_flag = false;
      }
      else
      {
        turn_counter--;
        drive(LEFT);
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

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
