/*
 * MASKOR EV3 BOBB3E NODE
 *
 * Copyright (c) 2017 - Marcel Stüttgen 
 * mail:stuettgen@fh-aachen.de
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <maskor_ev3/maskor_ev3.h>
//#include <maskor_ev3/motor.h>

// EV3 STUFF
//maskor_ev3::motor left_motor = maskor_ev3::OUTPUT_B;
//maskor_ev3::motor right_motor = maskor_ev3::OUTPUT_C;//randomly initialised 


// ROS STUFF
ros::NodeHandle  nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_msg;

char base_link[] = "/base_link";
char odom[] = "/odom";
char rosSrvrIp[] = "127.0.0.1";

void cmd_velCb(const geometry_msgs::Twist& twist_msg) {
  printf("Received cmd_vel message\n");
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_velCb );
ros::Publisher odom_pub("odom", &odom_msg);

void publish_odom(){
    odom_pub.publish(&odom_msg);
}


void publish_tf() {
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = 1.0; 
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0; 
  t.transform.rotation.z = 0.0; 
  t.transform.rotation.w = 1.0;  
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
}


int main()
{
  nh.initNode(rosSrvrIp);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  broadcaster.init(nh);

  while(1) {
    usleep(10000); //microseconds
    publish_odom();
    publish_tf();
    nh.spinOnce();
  }
}
