/*
 * MASKOR EV3 BOBB3E NODE
 *
 * Copyright (c) 2017 
 * Marcel Stüttgen 
 * stuettgen@fh-aachen.de
 *
 * Patrick Hannak 
 * patrick.hannak@alumni.fh-aachen.de
 */

#define _DEBUG
#define _OFFLINETEST

#include <stdio.h>
#include <maskor_ev3/maskor_ev3.h>

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <maskor_ev3_msgs/ColorSensor.h>
#include <maskor_ev3_msgs/GyroSensor.h>
#include <maskor_ev3_msgs/InfraredSensor.h>
#include <maskor_ev3_msgs/TouchSensor.h>
#include <maskor_ev3_msgs/UltrasonicSensor.h>


// FUNCTION DECLARATIONS

void cmd_velCb(const geometry_msgs::Twist& cmd);
void calc_odometry();
void publish_test_messages();

#ifndef _OFFLINETEST
void init_motors();
void init_sensors();
void motortest();
#endif


// ROS STUFF
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_msg;
maskor_ev3_msgs::ColorSensor color_sensor_msg;
maskor_ev3_msgs::GyroSensor gyro_sensor_msg;
maskor_ev3_msgs::TouchSensor touch_sensor_msg;
maskor_ev3_msgs::InfraredSensor infrared_sensor_msg;
maskor_ev3_msgs::UltrasonicSensor ultrasonic_sensor_msg;


ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_velCb );
ros::Publisher odom_pub("/bobb3e/odom", &odom_msg);
ros::Publisher color_sensor_pub("/bobb3e/color_sensor", &color_sensor_msg);
ros::Publisher gyro_sensor_pub("/bobb3e/gyro_sensor", &gyro_sensor_msg);
ros::Publisher touch_sensor_pub("/bobb3e/touch_sensor", &touch_sensor_msg);
ros::Publisher infrared_sensor_pub("/bobb3e/infrared_sensor", &infrared_sensor_msg);
ros::Publisher ultrasonic_sensor_pub("/bobb3e/ultrasonic_sensor", &ultrasonic_sensor_msg);



//global variables
const float deg2rad = M_PI/180.0;
char base_link[] = "/base_link";
char odom[] = "/odom";
//char rosSrvrIp[] = "10.42.0.1";
char rosSrvrIp[] = "127.0.0.1";

double left_motor_speed=0.0;
double right_motor_speed=0.0;
double lift_motor_speed=0.0;

bool lift_rot_flag = true;

int lift_rot_limit = 0;
int wheel_encoder_current_pos[2] = {0,0};
int wheel_encoder_prev_pos[2] = {0,0};
int dl = 0;
int dr = 0;
float trans_x = 0.0;
float trans_y = 0.0;
float theta = 0.0;
float vx = 0.0;
float wt = 0.0;
float vl = 0.0;
float vr = 0.0; 
float wheelbase = 0.12;
float wheelradius = 0.03;

#ifndef _OFFLINETEST
//Init motors
maskor_ev3::motor lift_motor(maskor_ev3::OUTPUT_A);
maskor_ev3::motor left_motor(maskor_ev3::OUTPUT_B);
maskor_ev3::motor right_motor(maskor_ev3::OUTPUT_C);
#endif


/*vx=velocity of centroid, wt=angular velocity of cenintroid, 
 vr,vl= velocity of wheels,L=distance between wheels R = radius of wheel*/


void cmd_velCb(const geometry_msgs::Twist& cmd) {

#ifdef _DEBUG
  printf("cmd_velCb(const geometry_msgs::Twist& cmd)\n");
#endif

  //set value to left motor speed
  if (cmd.linear.x != 0) {
    left_motor_speed = -cmd.linear.x;
  }
  else {
    left_motor_speed = 0.0;
  }

  //set value to right motor speed
  if (cmd.linear.x != 0) {
    right_motor_speed = -cmd.linear.x;
  }
  else {
    right_motor_speed = 0.0;
  }

 //set value to lift
  printf("Rot_limit: %d\n", lift_rot_limit);
  //lift_rot_limit++;

  if (lift_rot_limit == 0 && lift_rot_limit < 15 )
    lift_rot_flag = true;
  else if ( lift_rot_limit >= 15 )
    lift_rot_flag = false;
  
  if (lift_rot_flag == true)
    {
      if (cmd.linear.z !=0) {
	lift_motor_speed = cmd.linear.z;
	lift_rot_limit++;
      }
      else{
	lift_motor_speed = 0.0;
      }
    }
  else if (lift_rot_flag == false)
    {
      if (cmd.linear.z !=0){
	lift_motor_speed = -cmd.linear.z;
	lift_rot_limit--;
      }
      else{
	lift_motor_speed = 0.0;
      }
   }
  //left_motor.set_speed_sp(1.0);//setting up speed for the left motor 
  //left_motor.set_command("run-forever");
}

//function to calculate the odometry 
void calc_odometry() {

#ifdef _DEBUG
  printf("calc_odometry()\n");
#endif
 
#ifndef _OFFLINETEST
 //get current wheel positions  
  wheel_encoder_current_pos[0] = left_motor.position(); 
  wheel_encoder_current_pos[1] = right_motor.position();
#endif

  //obtaining angle rotated by left and right wheels
  dl = wheel_encoder_current_pos[0] - wheel_encoder_prev_pos[0];
  dr = wheel_encoder_current_pos[1] - wheel_encoder_prev_pos[1];

  //calc how far the robot has travelled
  trans_x += cos(theta) * (wheelradius * deg2rad) * (dl+dr)/2.0;
  trans_y += sin(theta) * (wheelradius * deg2rad) * (dl+dr)/2.0;

  //save current values for next iteration
  wheel_encoder_prev_pos[0] = wheel_encoder_current_pos[0];
  wheel_encoder_prev_pos[1] = wheel_encoder_current_pos[1];

#ifndef _OFFLINETEST
  //motor.speed() returns speed in rad/s
  vl = left_motor.speed();
  vr = right_motor.speed();
#endif

  //TODO: what is happening here??      
  //remmapping linear and angular velocity of centroid from vl, vr
  vx = (vl+vr) / 2*(wheelradius * deg2rad);
  wt = (vl-vr) / wheelbase * (wheelradius * deg2rad);

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);

  //first, we'll publish the transform over tf
  odom_tf.header.stamp = nh.now();
  odom_tf.header.frame_id = odom;
  odom_tf.child_frame_id = base_link;
  odom_tf.transform.translation.x = trans_x;
  odom_tf.transform.translation.y = trans_y;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_quat;
  broadcaster.sendTransform(odom_tf);

  //publish corresponding odom msg
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = odom;
  odom_msg.child_frame_id = base_link;
  odom_msg.pose.pose.position.x = trans_x; 
  odom_msg.pose.pose.position.y = trans_y;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = 0;
  odom_msg.twist.twist.angular.z = wt;
  odom_pub.publish(&odom_msg);
} 


void publish_test_messages() {
  printf("publish_test_messages()\n");

  color_sensor_msg.header.stamp = nh.now();
  color_sensor_msg.header.frame_id = "color_sensor_link";
  color_sensor_msg.color = 3;
  color_sensor_pub.publish(&color_sensor_msg);

  gyro_sensor_msg.header.stamp = nh.now();
  gyro_sensor_msg.header.frame_id = "gyro_sensor_link";
  gyro_sensor_msg.angle = 180;
  gyro_sensor_msg.rotational_speed = 3;
  gyro_sensor_pub.publish(&gyro_sensor_msg);

  touch_sensor_msg.header.stamp = nh.now();
  touch_sensor_msg.header.frame_id = "touch_sensor_link";
  touch_sensor_msg.state = 0;
  touch_sensor_pub.publish(&touch_sensor_msg);

  infrared_sensor_msg.header.stamp = nh.now();
  infrared_sensor_msg.header.frame_id = "infrared_sensor_link";
  infrared_sensor_msg.proximity = 0;
  infrared_sensor_pub.publish(&infrared_sensor_msg);

  ultrasonic_sensor_msg.header.stamp = nh.now();
  ultrasonic_sensor_msg.header.frame_id = "ultrasonic_sensor_link";
  ultrasonic_sensor_msg.distance = 0;
  ultrasonic_sensor_pub.publish(&ultrasonic_sensor_msg);

}

#ifndef _OFFLINETEST
void init_motors() {
  printf("Init Motors...\n");

  //init motor position
  left_motor.set_position(0);
  right_motor.set_position(0);
  lift_motor.set_position(0);

  /*
  //configuring the motors
  left_motor.reset();
  left_motor.set_position(0);
  left_motor.set_speed_regulation_enabled("on");
  */
}

void init_sensors() {
  printf("Init Sensors...\n");

 //init sensors
  maskor_ev3::sensor s(maskor_ev3::INPUT_4); 
}

void motor_test() {
      //print values
      // printf("sensor value: %d\n", s.value());
      printf("left_motor_position: %d\n", left_motor.position());
      printf("left_motor_speed: %d\n", -left_motor.speed_sp());
      printf("right_motor_position: %d\n", right_motor.position());
      printf("right_motor_speed: %d\n", -right_motor.speed_sp());

      printf("fork_motor_position: %d\n", lift_motor.position());
      printf("fork_motor_speed: %d\n", lift_motor.speed_sp());      
      printf("\n\n\n");

      //set speed     
      left_motor.set_speed_sp(left_motor_speed);
      left_motor.set_command("run-forever");
      right_motor.set_speed_sp(right_motor_speed);
      right_motor.set_command("run-forever");

      //move lift
      lift_motor.set_speed_sp(lift_motor_speed);
      lift_motor.set_command("run-forever");
} 

#endif

void init_node() {
  printf("Init Node...\n");

  nh.initNode(rosSrvrIp);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  nh.advertise(color_sensor_pub); 
  nh.advertise(gyro_sensor_pub); 
  nh.advertise(touch_sensor_pub); 
  nh.advertise(infrared_sensor_pub);
  nh.advertise(ultrasonic_sensor_pub); 
 
  broadcaster.init(nh);
}

int main(int argc, char* argv[])
{
  init_node();
#ifndef _OFFLINETEST
  init_motors();
  init_sensors();
#endif

  while(1)
    {
      //ros stuff
      calc_odometry();
      publish_test_messages();
      usleep(100000); //microseconds
      nh.spinOnce(); // check for incoming messages
    }
 
  return 0;


}

