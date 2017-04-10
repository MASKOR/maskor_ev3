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
#include <sensor_msgs/JointState.h>
//#include <maskor_ev3_msgs/ColorSensor.h>
//#include <maskor_ev3_msgs/GyroSensor.h>
//#include <maskor_ev3_msgs/InfraredSensor.h>


// FUNCTION DECLARATIONS
void cmd_velCb(const geometry_msgs::Twist& cmd);
void calc_odometry();
void publish_test_messages();
void publish_joint_states();
double calc_fork_lift_link_position(double arm_position);

#ifndef _OFFLINETEST
void init_motors();
void init_sensors();
#endif


// ROS STUFF
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster broadcaster;
nav_msgs::Odometry odom_msg;
sensor_msgs::JointState joint_state_msg;
//maskor_ev3_msgs::ColorSensor color_sensor_msg;
//maskor_ev3_msgs::GyroSensor gyro_sensor_msg;
//maskor_ev3_msgs::InfraredSensor infrared_sensor_msg;


ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_velCb );
ros::Publisher odom_pub("/odom", &odom_msg);
ros::Publisher joint_state_pub("/joint_states", &joint_state_msg);
//ros::Publisher color_sensor_pub("/bobb3e/color_sensor", &color_sensor_msg);
//ros::Publisher gyro_sensor_pub("/bobb3e/gyro_sensor", &gyro_sensor_msg);
//ros::Publisher infrared_sensor_pub("/bobb3e/infrared_sensor", &infrared_sensor_msg);

#ifndef _OFFLINETEST
//motors
maskor_ev3::motor lift_motor(maskor_ev3::OUTPUT_A);
maskor_ev3::motor left_motor(maskor_ev3::OUTPUT_B);
maskor_ev3::motor right_motor(maskor_ev3::OUTPUT_C);
//sensors
maskor_ev3::sensor gyro_sensor(maskor_ev3::INPUT_4);
#endif




//global variables
const float deg2rad = M_PI/180.0;
char base_link[] = "/base_footprint";
char odom[] = "/odom";
//char rosSrvrIp[] = "10.42.0.1";
//char rosSrvrIp[] = "149.201.178.169";
char rosSrvrIp[] = "127.0.0.1";

double left_motor_speed=0.0;
double right_motor_speed=0.0;
double lift_motor_speed=0.0;

bool lift_rot_flag = true;

float t=0; // simulated time for sinus
double pos_min = 0;
double pos_max = 0;

int lift_rot_limit = 0;
int wheel_encoder_current_pos[2] = {0,0};
int wheel_encoder_prev_pos[2] = {0,0};
int dl = 0;
int dr = 0;
int k = 0;
int q = 0;
float trans_x = 0.0;
float trans_y = 0.0;
float theta = 0.0;
float t_offset = 0.0;
float vx = 0.0;
float wt = 0.0;
float vl = 0.0;
float vr = 0.0; 
float wheelbase = 0.12;
float wheelradius = 0.03;

enum {
   LEFT_FRONT_WHEEL,
   LEFT_REAR_WHEEL,
   RIGHT_FRONT_WHEEL, 
   RIGHT_REAR_WHEEL,
   FORK_LIFT,
   RIGHT_ARM_LINK,
   LEFT_ARM_LINK,
   NUM_JOINTS
};
char *joint_names[] = {"base_link_to_left_front_wheel", 
		       "base_link_to_left_rear_wheel",
		       "base_link_to_right_front_wheel",       
		       "base_link_to_right_rear_wheel",       
		       "base_link_to_fork_lift_link",
		       "base_link_to_right_fork_arm",
		       "base_link_to_left_fork_arm"};


double joint_positions[NUM_JOINTS];
double joint_velocities[NUM_JOINTS];
double joint_efforts[NUM_JOINTS];


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
  //calculating gyro
  theta = -gyro_sensor.value()*deg2rad - t_offset;

  //making sure the angle lies in [-π,π]  
  theta = theta/deg2rad;
  if(theta>0)
    {
      k = theta/360;
      theta = theta - k*360;
      if(theta>0&&theta<90)
	q =1; 
      if(theta>90&&theta<180)
	q = 2;
      if(theta>180&&theta<270)
	q = 3;
      if(theta>270&&theta<360)
	q = 4;
      if(q==1||q==2)
	theta = theta + 0;
      if(q==4||q==3)
	theta = -360 + theta ;
    }

  if(theta<0)
    {
      k = -theta/360;
      theta = theta + k*360;
      if(theta<0 && theta>-90)
	q =4; 
      if(theta<-90&&theta>-180)
	q = 3;
      if(theta<-180&&theta>-270)
	q = 2;
      if(theta<-270&&theta>-360)
	q = 1;
      if(q==4||q==3)
	theta = theta + 0;
      if(q==1||q==2)
	theta = 360 + theta ;
    }

  theta*=deg2rad;
  
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

  /*vx=velocity of centroid, wt=angular velocity of cenintroid, 
    vr,vl= velocity of wheels,L=distance between wheels R = radius of wheel*/
  //remmapping linear and angular velocity of centroid from vl, vr
  vx = (vl+vr) / 2*(wheelradius * deg2rad);
  wt = (vl-vr) / wheelbase * (wheelradius * deg2rad);

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta);

  //publish odom TF
  odom_tf.header.stamp = nh.now();
  odom_tf.header.frame_id = odom;
  odom_tf.child_frame_id = base_link;
  odom_tf.transform.translation.x = trans_x;
  odom_tf.transform.translation.y = trans_y;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_quat;
  broadcaster.sendTransform(odom_tf);

  //publish odom msg
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
  /*
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

  // touch_sensor_msg.header.stamp = nh.now();
  // touch_sensor_msg.header.frame_id = "touch_sensor_link";
  // touch_sensor_msg.state = 0;
  // touch_sensor_pub.publish(&touch_sensor_msg);

  infrared_sensor_msg.header.stamp = nh.now();
  infrared_sensor_msg.header.frame_id = "infrared_sensor_link";
  infrared_sensor_msg.proximity = 0;
  infrared_sensor_pub.publish(&infrared_sensor_msg);

  // ultrasonic_sensor_msg.header.stamp = nh.now();
  // ultrasonic_sensor_msg.header.frame_id = "ultrasonic_sensor_link";
  // ultrasonic_sensor_msg.distance = 0;
  // ultrasonic_sensor_pub.publish(&ultrasonic_sensor_msg);
  */
}

void publish_joint_states() {
  printf("publish_joint_states()\n");
 
  //TODO: read joint states from motors
// # The state of each joint (revolute or prismatic) is defined by:
// #  * the position of the joint (rad or m),
// #  * the velocity of the joint (rad/s or m/s) and 
// #  * the effort that is applied in the joint (Nm or N).  
#ifndef _OFFLINETEST
  joint_positions[RIGHT_FRONT_WHEEL] = right_motor.position(); //deg or rad??
  joint_positions[RIGHT_REAR_WHEEL] = right_motor.position(); 
  joint_positions[LEFT_FRONT_WHEEL] = left_motor.position(); //deg or rad??
  joint_positions[LEFT_REAR_WHEEL] = left_motor.position(); 
  joint_positions[LEFT_ARM_LINK] = lift_motor.position();
  joint_positions[RIGHT_ARM_LINK] = lift_motor.position();
  joint_positions[FORK_LIFT] = 0;

  joint_velocities[RIGHT_FRONT_WHEEL] = right_motor.speed();
  joint_velocities[RIGHT_REAR_WHEEL] = right_motor.speed(); 
  joint_velocities[LEFT_FRONT_WHEEL] = left_motor.speed();
  joint_velocities[LEFT_REAR_WHEEL] = left_motor.speed(); 
  joint_velocities[FORK_LIFT] = 0;
  joint_velocities[LEFT_ARM_LINK] = lift_motor.speed();
  joint_velocities[RIGHT_ARM_LINK] = lift_motor.speed();

  joint_efforts[RIGHT_FRONT_WHEEL] = 0;
  joint_efforts[RIGHT_REAR_WHEEL] = 0; 
  joint_efforts[LEFT_FRONT_WHEEL] = 0;
  joint_efforts[LEFT_REAR_WHEEL] = 0;
  joint_efforts[FORK_LIFT] = 0;
  joint_efforts[LEFT_ARM_LINK] = 0;
  joint_efforts[RIGHT_ARM_LINK] = 0;
#else
  joint_positions[RIGHT_FRONT_WHEEL] = 0; 
  joint_positions[RIGHT_REAR_WHEEL] = 0; 
  joint_positions[LEFT_FRONT_WHEEL] = 0;
  joint_positions[LEFT_REAR_WHEEL] = 0; 
  joint_positions[LEFT_ARM_LINK] = (sin(t) * 1.1) - (3.14159/5);
  joint_positions[RIGHT_ARM_LINK] = joint_positions[LEFT_ARM_LINK];
  joint_positions[FORK_LIFT] = calc_fork_lift_link_position(joint_positions[LEFT_ARM_LINK]);

  printf("pos: %f\n",joint_positions[LEFT_ARM_LINK]);
  t+=0.05;
  if (t>3.14)
    t=0;

  if (joint_positions[LEFT_ARM_LINK] < pos_min)
    pos_min = joint_positions[LEFT_ARM_LINK];

  if(joint_positions[LEFT_ARM_LINK] > pos_max)
    pos_max = joint_positions[LEFT_ARM_LINK];

  printf("pos_min: %f\n",pos_min);
  printf("pos_max: %f\n",pos_max);


  joint_velocities[RIGHT_FRONT_WHEEL] = 0;
  joint_velocities[RIGHT_REAR_WHEEL] = 0; 
  joint_velocities[LEFT_FRONT_WHEEL] = 0;
  joint_velocities[LEFT_REAR_WHEEL] = 0; 
  joint_velocities[FORK_LIFT] = 0;
  joint_velocities[LEFT_ARM_LINK] = 1;
  joint_velocities[RIGHT_ARM_LINK] = 1;

  joint_efforts[RIGHT_FRONT_WHEEL] = 0;
  joint_efforts[RIGHT_REAR_WHEEL] = 0; 
  joint_efforts[LEFT_FRONT_WHEEL] = 0;
  joint_efforts[LEFT_REAR_WHEEL] = 0;
  joint_efforts[FORK_LIFT] = 0;
  joint_efforts[LEFT_ARM_LINK] = 0;
  joint_efforts[RIGHT_ARM_LINK] = 0;
#endif

  joint_state_msg.header.stamp = nh.now();
  joint_state_msg.header.frame_id = "/bobb3e";
  joint_state_msg.name_length = NUM_JOINTS;
  joint_state_msg.velocity_length = NUM_JOINTS;
  joint_state_msg.position_length = NUM_JOINTS; 
  joint_state_msg.effort_length = NUM_JOINTS;

  joint_state_msg.name = joint_names;
  joint_state_msg.position = joint_positions;
  joint_state_msg.velocity = joint_velocities;
  joint_state_msg.effort = joint_efforts;

  joint_state_pub.publish(&joint_state_msg);
}

double calc_fork_lift_link_position(double arm_position) {
  
  //linear projection of arm position [-100:100] to fork lift position [-0.025:0.1025]
  // values: try and error
  double arm_min = pos_max;
  double arm_max =  pos_min;
  double lift_min = -0.025;
  double lift_max =  0.11;

  //y=m*x+b
  double delta_x = arm_max - arm_min;
  double delta_y = lift_max - lift_min;
  double x = arm_position;
  double m = delta_y / delta_x;
  double b = lift_max - (m * arm_max);

  return  m * x + b;
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
}
#endif

void init_node() {
  printf("Init Node...\n");

  nh.initNode(rosSrvrIp);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  nh.advertise(joint_state_pub); 
  //nh.advertise(color_sensor_pub); 
  //nh.advertise(gyro_sensor_pub); 
  //nh.advertise(infrared_sensor_pub);
 
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
      publish_joint_states();
      usleep(100000); //microseconds
      nh.spinOnce(); // check for incoming messages
    }
 
  return 0;


}

