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

#include <stdio.h>
#include <maskor_ev3/maskor_ev3.h>

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// FUNCTION DECLARATIONS
void cmd_velCb(const geometry_msgs::Twist& cmd);
void calc_odometry();

// EV3 STUFF
maskor_ev3::motor left_motor(maskor_ev3::OUTPUT_C);
maskor_ev3::motor right_motor(maskor_ev3::OUTPUT_C);//randomly initialised 
maskor_ev3::infrared_sensor ir_sensor(maskor_ev3::INPUT_1);

// ROS STUFF
ros::NodeHandle nh;
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_msg;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_velCb );
ros::Publisher odom_pub("odom", &odom_msg);


//global variables
const float deg2rad = M_PI/180.0;
char base_link[] = "/base_link";
char odom[] = "/odom";
char rosSrvrIp[] = "10.42.0.1";

double left_motor_speed=0.0;
double right_motor_speed=0.0;
double lift_motor_speed=0.0;

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


/*vx=velocity of centroid, wt=angular velocity of cenintroid, 
 vr,vl= velocity of wheels,L=distance between wheels R = radius of wheel*/


void cmd_velCb(const geometry_msgs::Twist& cmd) {
  printf("received cmd_vel_msg\n");

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
  int rot_limit = 0;
  printf("Rot_limit: %d", rot_limit);
  
  if (rot_limit == 0 || rot_limit < 10)
    {
      if (cmd.linear.z !=0){
	lift_motor_speed = cmd.linear.z;
      }
      else{
	lift_motor_speed = 0.0;
      }
      rot_limit++;
    }
  else if (rot_limit >= 10)
    {
      if (cmd.linear.z !=0){
	lift_motor_speed = -cmd.linear.z;
      }
      else{
	lift_motor_speed = 0.0;
      }
      rot_limit--;
    }
  //left_motor.set_speed_sp(1.0);//setting up speed for the left motor 
  //left_motor.set_command("run-forever");
}


//function to calculate the odometry 
void calc_odometry() {

  //get current wheel positions  
  wheel_encoder_current_pos[0] = left_motor.position(); 
  wheel_encoder_current_pos[1] = right_motor.position();

  //obtaining angle rotated by left and right wheels
  dl = wheel_encoder_current_pos[0] - wheel_encoder_prev_pos[0];
  dr = wheel_encoder_current_pos[1] - wheel_encoder_prev_pos[1];

  //calc how far the robot has travelled
  trans_x += cos(theta) * (wheelradius * deg2rad) * (dl+dr)/2.0;
  trans_y += sin(theta) * (wheelradius * deg2rad) * (dl+dr)/2.0;

  //save current values for next iteration
  wheel_encoder_prev_pos[0] = wheel_encoder_current_pos[0];
  wheel_encoder_prev_pos[1] = wheel_encoder_current_pos[1];

  //motor.speed() returns speed in rad/s
  vl = left_motor.speed();
  vr = right_motor.speed();

  //TODO: what is happening here??      
  //remmapping linear and angular velocity of centroid from vl, vr
  vx = (vl+vr) / 2*(wheelradius * deg2rad);
  wt = (vl-vr) / wheelbase * (wheelradius * deg2rad);

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);

  //first, we'll publish the transform over tf
  odom_tf.header.stamp = nh.now();;
  odom_tf.header.frame_id = odom;
  odom_tf.child_frame_id = base_link;
  odom_tf.transform.translation.x = trans_x;
  odom_tf.transform.translation.y = trans_y;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_quat;
  broadcaster.sendTransform(odom_tf);
} 


int main(int argc, char* argv[])
{
  printf("Init Node...\n");

  nh.initNode(rosSrvrIp);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  broadcaster.init(nh);
    
  printf("Init Motors...\n");

  //init motors
  maskor_ev3::motor lift_motor(maskor_ev3::OUTPUT_A);
  maskor_ev3::motor left_motor(maskor_ev3::OUTPUT_B);
  maskor_ev3::motor right_motor(maskor_ev3::OUTPUT_C);
  left_motor.set_position(0);
  right_motor.set_position(0);
  lift_motor.set_position(0);

  /*
  //configuring the motors
  left_motor.reset();
  left_motor.set_position(0);
  left_motor.set_speed_regulation_enabled("on");
  */

  printf("Init Sensors...\n");
  
  //init sensors
  maskor_ev3::sensor s(maskor_ev3::INPUT_4);

  
  while(1)
    {
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
     
      //ros stuff
      calc_odometry();
      usleep(10000); //microseconds
      nh.spinOnce(); // check for incoming messages
    }
 
  return 0;


}

