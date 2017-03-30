/*Note : The ev3 listens to the topic turtlebot_teleop/cmd_vel which is published by the turtlebot_teleop_key.   It can be obtained by running the following command in the host,
  $ rosrun turtlebot_teleop turtlebot_teleop_key */

//Header files required to run ROS
#include <ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

//Header files required to run EV3dev
#include <iostream>
#include "ev3dev.h"
#include <thread>
#include <string.h>
#include <math.h>
#include <chrono>
//#include <errno.h>

using namespace std;
using namespace ev3dev;//
int k = 0, q = 0;

ros::NodeHandle  nh;
char rosSrvrIp[] = "10.42.0.1";//IP address of the master

double left_motor_speed=0.0;
double right_motor_speed=0.0;

//motor left_motor("outB");
//motor left_motor;

//sensor s = sensor(INPUT_4);

void cmd_vel_cb(const geometry_msgs::Twist& cmd) {
  printf("received cmd_vel_msg\n");

  //set value to left motor speed
  if (cmd.linear.x != 0) {
    left_motor_speed = cmd.linear.x;
  }
  else {
    left_motor_speed = 0.0;
  }

  //set value to right motor speed
  if (cmd.linear.x != 0) {
    right_motor_speed = cmd.linear.x;
  }
  else {
    right_motor_speed = 0.0;
  }
  
  //left_motor.set_speed_sp(1.0);//setting up speed for the left motor 
  //left_motor.set_command("run-forever");
}

ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", cmd_vel_cb );

int main(int argc, char* argv[])
{      
  printf("Checkpoint1\n");

  /*
  //Initialising motors and checking if connected 
  left_motor = motor("outB");
    
  //configuring the motors
  left_motor.reset();
  left_motor.set_position(0);
  left_motor.set_speed_regulation_enabled("on");
  */

  //init left and right motor
  //string left_motor_port ="outB";
  motor left_motor("outB");
  left_motor.set_position(0);

  //string right_motor_port ="outC";
  motor right_motor("outC");
  right_motor.set_position(0);
 
  //init forklift
  //string fork_motor_port ="outA";
  motor lift("outA");
  lift.set_position(0);
  
  //init a sensor
  string sensor_port = "in4";
  sensor s = sensor(sensor_port);
  printf("Checkpoint2");

  //rosSrvrIp = (argv[1]);
  nh.initNode(rosSrvrIp);//setup proxy 
  nh.subscribe(cmd_sub);//subscribing to the topic to receive velocity commands
  
  while(1)
    {
      //print values
      // printf("sensor value: %d\n", s.value());
      printf("left_motor_position: %d\n", left_motor.position());
      printf("left_motor_speed: %f\n", left_motor_speed);
      printf("right_motor_position: %d\n", right_motor.position());
      printf("right_motor_speed: %f\n", right_motor_speed);
      printf("\n\n\n");

      //set speed
      left_motor.set_speed_sp(left_motor_speed);
      left_motor.set_command("run-forever");
      right_motor.set_speed_sp(right_motor_speed);
      right_motor.set_command("run-forever");

      //ros stuff
      usleep(10000); //microseconds
      nh.spinOnce(); // check for incoming messages
    }
 
  return 0;
}

