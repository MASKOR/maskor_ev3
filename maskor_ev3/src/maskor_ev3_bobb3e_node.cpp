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

//#define _DEBUG
//#define _OFFLINETEST

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
#include <maskor_ev3_msgs/ColorSensor.h>
#include <maskor_ev3_msgs/GyroSensor.h>
#include <maskor_ev3_msgs/InfraredSensor.h>


// FUNCTION DECLARATIONS
void cmd_velCb(const geometry_msgs::Twist& cmd);
void calc_odometry();
void publish_sensor_messages();
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
maskor_ev3_msgs::ColorSensor color_sensor_msg;
maskor_ev3_msgs::GyroSensor gyro_sensor_msg;
maskor_ev3_msgs::InfraredSensor infrared_sensor_msg;


ros::NodeHandle nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_velCb );
ros::Publisher odom_pub("/odom", &odom_msg);
ros::Publisher joint_state_pub("/joint_states", &joint_state_msg);
ros::Publisher color_sensor_pub("/bobb3e/color_sensor", &color_sensor_msg);
ros::Publisher gyro_sensor_pub("/bobb3e/gyro_sensor", &gyro_sensor_msg);
ros::Publisher infrared_sensor_pub("/bobb3e/infrared_sensor", &infrared_sensor_msg);


#ifndef _OFFLINETEST
//motors
maskor_ev3::motor lift_motor(maskor_ev3::OUTPUT_A);
maskor_ev3::motor left_motor(maskor_ev3::OUTPUT_B);
maskor_ev3::motor right_motor(maskor_ev3::OUTPUT_C);
//sensors
maskor_ev3::color_sensor color_sensor(maskor_ev3::INPUT_1);
maskor_ev3::infrared_sensor ir_sensor(maskor_ev3::INPUT_3);
//maskor_ev3::gyro_sensor gyro_sensor(maskor_ev3::INPUT_4);
#endif


//--------------------------------------------------------global variables--------------------------------------------------------

//motor/wheel translation

//small gear z1=12
//big gear z2=20
//=> translation (z1/z2): 12/20 = 0.6
//=> one cycle motor = 0.6 cycles wheels
//=>one cycle motor = 0.6*2*PI*1.5cm(r_wheels) = ((9*pi)/5)cm ~~ 5.655cm

const float deg2rad = M_PI/180.0;
char base_link[] = "/base_footprint";
char odom[] = "/odom";


#ifndef _OFFLINETEST
char rosSrvrIp[] = "149.201.37.134";
#else
//char rosSrvrIp[] = "149.201.178.169";
char rosSrvrIp[] = "127.0.0.1";
#endif

double left_motor_speed=0.0;
double right_motor_speed=0.0;
double lift_motor_speed=0.0;

float t=0; // simulated time for sinus
double pos_min = 0;
double pos_max = M_PI/4;

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
float wheelbase = 0.95;
float wheelradius = 0.015;

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

/*
char* joint_names[NUM_JOINTS];
joint_names[LEFT_FRONT_WHEEL] ="base_link_to_left_front_wheel";
joint_names[LEFT_REAR_WHEEL]  ="base_link_to_left_rear_wheel";
joint_names[RIGHT_FRONT_WHEEL]="base_link_to_right_front_wheel";
joint_names[RIGHT_REAR_WHEEL] ="base_link_to_right_rear_wheel";
joint_names[FORK_LIFT]        ="base_link_to_fork_lift_link";
joint_names[RIGHT_ARM_LINK]   ="base_link_to_left_fork_arm";
joint_names[LEFT_ARM_LINK]    ="base_link_to_left_fork_arm";
*/

double joint_positions[NUM_JOINTS];
double joint_velocities[NUM_JOINTS];
double joint_efforts[NUM_JOINTS];

void set_motor_speed()
{
#ifndef _OFFLINETEST
  left_motor.set_speed_sp(left_motor_speed);
  left_motor.set_command("run-forever");
  right_motor.set_speed_sp(right_motor_speed);
  right_motor.set_command("run-forever");
  //lift_motor.set_speed_sp(lift_motor_speed);
  //lift_motor.set_command("run-forever");
#endif
}

//-----------------------------Turning Functions -----------------------------------------

//distance between colorsensor and wheelbase = 9cm
//one motorcycle ~~ 6cm
//9/6 = 1.5 => 1.5 * 2*PI = 540 degrees
//turn of 90 degrees = 2 motorcycles => 720 degrees

/*
//Absolute Values ------> Better with gyro
void turn_left_abs() {
  left_motor.set_position_sp(-540);
  right_motor.set_position_sp(-540);

  left_motor.set_command("run-to-rel-pos");
  right_motor.set_command("run-to-rel-pos");

  usleep(5000000);

  left_motor.set_position_sp(720);
  right_motor.set_position_sp(-720);

  left_motor.set_command("run-to-rel-pos");
  right_motor.set_command("run-to-rel-pos");

  usleep(6000000);
}

void turn_right_abs() {
  left_motor.set_position_sp(-540);
  right_motor.set_position_sp(-540);

  left_motor.set_command("run-to-rel-pos");
  right_motor.set_command("run-to-rel-pos");

  usleep(5000000);

  left_motor.set_position_sp(-720);
  right_motor.set_position_sp(720);

  left_motor.set_command("run-to-rel-pos");
  right_motor.set_command("run-to-rel-pos");

  usleep(6000000);
}
*/

//Turning functions with gyro
/*void turn_left() {
#ifndef _OFFLINETEST
  int cur_deg = gyro_sensor.value();

  left_motor.set_position_sp(-540);
  right_motor.set_position_sp(-540);

  left_motor.set_speed_sp(100);
  right_motor.set_speed_sp(-100);

  left_motor.set_command("run-to-rel-pos");
  right_motor.set_command("run-to-rel-pos");

  usleep(5000000);

  while (gyro_sensor.value() > cur_deg-90)
    {
      printf("gyro value: %d \n", gyro_sensor.value());
      left_motor.set_command("run-forever");
      right_motor.set_command("run-forever");
      calc_odometry();
      publish_joint_states();
    }

  left_motor.set_command("stop");
  right_motor.set_command("stop");
#endif
}
*/

/*void turn_right() {
#ifndef _OFFLINETEST
int cur_deg = gyro_sensor.value();

  left_motor.set_position_sp(-540);
  right_motor.set_position_sp(-540);

  left_motor.set_speed_sp(-100);
  right_motor.set_speed_sp(100);

  left_motor.set_command("run-to-rel-pos");
  right_motor.set_command("run-to-rel-pos");

  usleep(5000000);

  while (gyro_sensor.value() < cur_deg+90)
    {
      printf("gyro value: %d \n", gyro_sensor.value());
      left_motor.set_command("run-forever");
      right_motor.set_command("run-forever");
      calc_odometry();
      publish_joint_states();
    }

  left_motor.set_command("stop");
  right_motor.set_command("stop");
#endif
}
*/
//-------------------------------------------------------------lift-functions---------------------------------------------------

//Lift moves upwards until the IR "sees" the fork, it then will move approx. to the ground

void reset_lift(){
#ifndef _OFFLINETEST

  printf("Resetting lift....\n");

  lift_motor.set_speed_sp(100);

  while(ir_sensor.value() > 30)
    {
      lift_motor.set_command("run-forever");
    }

  lift_motor.set_command("stop");
  lift_motor.set_time_sp(1500);
  lift_motor.set_speed_sp(-100);
  lift_motor.set_command("run-timed");

  printf("Lift resetted! \n");
  usleep(2000000);
#endif
}

void lift_up(){
  #ifndef _OFFLINETEST

  lift_motor.set_position_sp(120);
  lift_motor.set_speed_sp(100);
  lift_motor.set_command("run-to-rel-pos");
  usleep(1000000);
#endif
}

void lift_down(){
  #ifndef _OFFLINETEST

  lift_motor.set_position_sp(-120);
  lift_motor.set_speed_sp(100);
  lift_motor.set_command("run-to-rel-pos");
  usleep(1000000);
#endif
}

//-------------------------------------------------------------cmd_vel callback------------------------------------------------

//Motors are upside down => add a minus to move in right direction

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
}

//------------------------------------------------------------------------------calculating fork lift position----------------------------------------------------

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
#ifndef _OFFLINETEST
  double x = (lift_motor.position())%360;
#else
  double x = 15; //dummy value
#endif
  double m = delta_y / delta_x;
  double b = lift_max - (m * arm_max);

  return  m * x + b;
}

//----------------------------------------------------------------------calculating odometry-------------------------------------------------------------------------

//function to calculate the odometry
void calc_odometry() {

#ifdef _DEBUG
  printf("calc_odometry()\n");
#endif

#ifndef _OFFLINETEST
  //calculating gyro
  //theta = -gyro_sensor.value()*deg2rad - t_offset;

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

//------------------------------------------------------------------------------publishing sensor messages for offline debug----------------------------------------------------

void publish_sensor_messages() {

  printf("publish_sensor_messages()\n");

  color_sensor_msg.header.stamp = nh.now();
  color_sensor_msg.header.frame_id = "color_sensor_link";
  color_sensor_msg.color = color_sensor.value();
  color_sensor_pub.publish(&color_sensor_msg);

  //gyro_sensor_msg.header.stamp = nh.now();
  //gyro_sensor_msg.header.frame_id = "gyro_sensor_link";
  //gyro_sensor_msg.angle = gyro_sensor.value(0);
  //gyro_sensor_msg.rotational_speed = gyro_sensor.value(1);
  //gyro_sensor_pub.publish(&gyro_sensor_msg);

  infrared_sensor_msg.header.stamp = nh.now();
  infrared_sensor_msg.header.frame_id = "infrared_sensor_link";
  infrared_sensor_msg.proximity = ir_sensor.value();
  infrared_sensor_pub.publish(&infrared_sensor_msg);
}

//------------------------------------------------------------------------------publishing joint states----------------------------------------------------

void publish_joint_states() {
  // printf("publish_joint_states()\n");

  //TODO: read joint states from motors
// # The state of each joint (revolute or prismatic) is defined by:
// #  * the position of the joint (rad or m),
// #  * the velocity of the joint (rad/s or m/s) and
// #  * the effort that is applied in the joint (Nm or N).
#ifndef _OFFLINETEST
  /*
  joint_positions[RIGHT_FRONT_WHEEL] = right_motor.position()*deg2rad; //deg or rad??
  joint_positions[RIGHT_REAR_WHEEL] = right_motor.position()*deg2rad;
  joint_positions[LEFT_FRONT_WHEEL] = left_motor.position()*deg2rad; //deg or rad??
  joint_positions[LEFT_REAR_WHEEL] = left_motor.position()*deg2rad;
  joint_positions[LEFT_ARM_LINK] = lift_motor.position()*deg2rad;
  joint_positions[RIGHT_ARM_LINK] = lift_motor.position()*deg2rad;
  joint_positions[FORK_LIFT] = calc_fork_lift_link_position(joint_positions[LEFT_ARM_LINK])*deg2rad;;
*/

  joint_positions[RIGHT_FRONT_WHEEL] = right_motor.position(); //deg or rad??
  joint_positions[RIGHT_REAR_WHEEL] = right_motor.position();
  joint_positions[LEFT_FRONT_WHEEL] = left_motor.position(); //deg or rad??
  joint_positions[LEFT_REAR_WHEEL] = left_motor.position();
  joint_positions[LEFT_ARM_LINK] = lift_motor.position()*deg2rad;
  joint_positions[RIGHT_ARM_LINK] = lift_motor.position()*deg2rad;
  joint_positions[FORK_LIFT] = calc_fork_lift_link_position(joint_positions[LEFT_ARM_LINK])*deg2rad;;

  /*
  printf("pos Lift: %f\n", lift_motor.position()*deg2rad);
  printf("vel Lift: %f\n", lift_motor.speed()*deg2rad);
  */

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


//------------------------------------------------------------------------------Init functions-------------------------------------------------------------------

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
  //gyro_sensor.rate();
  //gyro_sensor.g_a();
  ir_sensor.proximity();
  color_sensor.color();
}
#endif

void init_node(char ip[]) {
  printf("Init Node...\n");

  nh.initNode(ip);
  nh.subscribe(cmd_vel_sub);
  nh.advertise(odom_pub);
  nh.advertise(joint_state_pub);
  nh.advertise(color_sensor_pub);
  //nh.advertise(gyro_sensor_pub);
  nh.advertise(infrared_sensor_pub);

  broadcaster.init(nh);
}

int main(int argc, char* argv[])
{
  if (argc == 2) {
    init_node(argv[1]);
  }
  else {
    init_node("10.42.0.1");
  }
    
#ifndef _OFFLINETEST
  init_motors();
  init_sensors();
#endif

  usleep(1000000);

  printf("Entering loop....\n");

  while(1)
    {
      //ros stuff
      set_motor_speed();
      calc_odometry();
      publish_sensor_messages();
      publish_joint_states();
      usleep(100000); //microseconds
      nh.spinOnce(); // check for incoming messages
    }

  return 0;
}
