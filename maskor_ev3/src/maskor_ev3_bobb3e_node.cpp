/*
 * MASKOR EV3 BOBB3E NODE
 *
 * Copyright (c) 2017
 * Marcel Stüttgen
 * stuettgen@fh-aachen.de
 *
 * Patrick Hannak
 * patrick.hannak@alumni.fh-aachen.de
 *
 * Dennis Miltz
 * dennis.miltz@alumni.fh-aachen.de
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
#include <sensor_msgs/JointState.h>
#include <maskor_ev3_msgs/ColorSensor.h>
#include <maskor_ev3_msgs/GyroSensor.h>
#include <maskor_ev3_msgs/InfraredSensor.h>


// FUNCTION DECLARATIONS
void cmd_velCb(const geometry_msgs::Twist& cmd);
void calc_odometry();
void publish_odometry_messages();
void publish_sensor_messages();
void publish_joint_states();

void init_motors();
void init_sensors();


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

char base_link[] = "/base_footprint";
char odom[] = "/odom";



//motors
maskor_ev3::motor lift_motor(maskor_ev3::OUTPUT_A);
maskor_ev3::motor left_motor(maskor_ev3::OUTPUT_B);
maskor_ev3::motor right_motor(maskor_ev3::OUTPUT_C);
//sensors
maskor_ev3::color_sensor color_sensor(maskor_ev3::INPUT_1);
maskor_ev3::infrared_sensor ir_sensor(maskor_ev3::INPUT_3);
//maskor_ev3::gyro_sensor gyro_sensor(maskor_ev3::INPUT_4);


const float ticks_per_rotation = 360.0;
const float deg2rad = M_PI/180.0;

double left_motor_speed=0.0;
double right_motor_speed=0.0;
double lift_motor_speed=0.0;

int wheel_encoder_current_pos[2] = {0,0};
int wheel_encoder_prev_pos[2] = {0,0};
ros::Time time_curr, time_prev;
int dl = 0;
int dr = 0;

float wheelradius = 0.016f; //without chain 0.015
float theta_curr, theta_prev= 0.0;
float x_prev, x_curr= 0.0;
float y_prev, y_curr=0.0;
float gearRatio = (12.0f/20.0f) * -1;
float axisDistance = 0.145;


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

static const char *joint_names_[] = {"base_link_to_left_front_wheel",
		       "base_link_to_left_rear_wheel",
		       "base_link_to_right_front_wheel",
		       "base_link_to_right_rear_wheel",
		       "base_link_to_fork_lift_link",
		       "base_link_to_right_fork_arm",
		       "base_link_to_left_fork_arm"};

char** joint_names=(char**)joint_names_;

double joint_positions[NUM_JOINTS];
double joint_velocities[NUM_JOINTS];
double joint_efforts[NUM_JOINTS];





void set_motor_speed()
{
  left_motor.set_speed_sp(left_motor_speed);
  left_motor.set_command("run-forever");
  right_motor.set_speed_sp(right_motor_speed);
  right_motor.set_command("run-forever");
  //lift_motor.set_speed_sp(lift_motor_speed);
  //lift_motor.set_command("run-forever");
}


void cmd_velCb(const geometry_msgs::Twist& cmd) {

  float x_ = cmd.linear.x;
  float rot_ = cmd.angular.z;
  float z_ = -cmd.linear.z;

  //0.15 m/s is the speed limit of the Ev3 Motor
  if(x_ > 0.15){
    x_ = 0.15;
  }
  else if(x_ < -0.15){
    x_ = -0.15;
  }

  if(rot_ > 0.15){
    rot_ = 0.15;
  }
  else if(rot_ < -0.15){
    rot_ = -0.15;
  }

  //magig calculate for magic
  float vr = 17.7 * ticks_per_rotation * (-1) * x_;
  float va = 10 *  ticks_per_rotation * (-1) * rot_;

  right_motor_speed = vr + va;
  left_motor_speed  = vr - va;

  lift_motor_speed = z_;

}


//----------------------------------------------------------------------calculating odometry-------------------------------------------------------------------------

void calc_odometry() {

  double delta_theta, delta_theta_grad = 0.0;

  time_curr = nh.now();

  //get current wheel positions
  wheel_encoder_current_pos[0] = (float)left_motor.position();
  wheel_encoder_current_pos[1] = (float)right_motor.position();

  dl = (wheel_encoder_current_pos[0] - wheel_encoder_prev_pos[0]);
  dr = (wheel_encoder_current_pos[1] - wheel_encoder_prev_pos[1]);

  dl *= gearRatio;
  dr *= gearRatio;

  delta_theta = ((dr * (2*M_PI*wheelradius/ticks_per_rotation)) - (dl * (2*M_PI*wheelradius/ticks_per_rotation)))/axisDistance;

  theta_curr = theta_prev + delta_theta;
  delta_theta_grad = delta_theta * deg2rad;

  x_curr = x_prev + (((dr*(2*M_PI*wheelradius/ticks_per_rotation)) + (dl*(2*M_PI*wheelradius/ticks_per_rotation)))/2)*cos(theta_curr);
  y_curr = y_prev + (((dr*(2*M_PI*wheelradius/ticks_per_rotation)) + (dl*(2*M_PI*wheelradius/ticks_per_rotation)))/2)*sin(theta_curr);


  double dt = time_curr.toSec() - time_prev.toSec();

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionFromYaw(theta_curr);

  //publish odom msg
  odom_msg.header.stamp = nh.now();
  odom_msg.header.frame_id = odom;
  odom_msg.child_frame_id = base_link;
  odom_msg.pose.pose.position.x = x_curr;
  odom_msg.pose.pose.position.y = y_curr;
  odom_msg.pose.pose.position.z = 0.0;
  odom_msg.pose.pose.orientation = odom_quat;
  odom_msg.twist.twist.linear.x = (x_curr - x_prev) / dt;
  odom_msg.twist.twist.linear.y = (y_curr - y_prev) / dt;
  odom_msg.twist.twist.angular.z = (theta_curr - theta_prev) / dt;
  odom_pub.publish(&odom_msg);

 //publish odom TF
  odom_tf.header.stamp = nh.now();
  odom_tf.header.frame_id = odom;
  odom_tf.child_frame_id = base_link;
  odom_tf.transform.translation.x = x_curr;
  odom_tf.transform.translation.y = y_curr;
  odom_tf.transform.translation.z = 0.0;
  odom_tf.transform.rotation = odom_quat;
  broadcaster.sendTransform(odom_tf);


  theta_prev = theta_curr;
  x_prev = x_curr;
  y_prev = y_curr;
  wheel_encoder_prev_pos[0] = wheel_encoder_current_pos[0];
  wheel_encoder_prev_pos[1] = wheel_encoder_current_pos[1];
  time_prev = time_curr;
}

//------------------------------------------------------------------------------publishing sensor messages for offline debug----------------------------------------------------

void publish_sensor_messages() {

  //printf("publish_sensor_messages()\n");

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

  joint_positions[RIGHT_FRONT_WHEEL] = (right_motor.position() % 360) * (2*M_PI/360.0) *-1; //wheel encoder position
  joint_positions[RIGHT_REAR_WHEEL] = (right_motor.position() % 360) * (2*M_PI/360.0)*-1;
  joint_positions[LEFT_FRONT_WHEEL] = (left_motor.position() % 360) * (2*M_PI/360.0)*-1;
  joint_positions[LEFT_REAR_WHEEL] = (left_motor.position() % 360) * (2*M_PI/360.0)*-1;
  joint_positions[LEFT_ARM_LINK] = 0;
  joint_positions[RIGHT_ARM_LINK] = 0;
  joint_positions[FORK_LIFT] = 0;

  joint_velocities[RIGHT_FRONT_WHEEL] = right_motor.speed(); //tacho count per second
  joint_velocities[RIGHT_REAR_WHEEL] = right_motor.speed();
  joint_velocities[LEFT_FRONT_WHEEL] = left_motor.speed();
  joint_velocities[LEFT_REAR_WHEEL] = left_motor.speed();
  joint_velocities[FORK_LIFT] = 0;
  joint_velocities[LEFT_ARM_LINK] = 0;
  joint_velocities[RIGHT_ARM_LINK] = 0;

  joint_efforts[RIGHT_FRONT_WHEEL] = 0;
  joint_efforts[RIGHT_REAR_WHEEL] = 0;
  joint_efforts[LEFT_FRONT_WHEEL] = 0;
  joint_efforts[LEFT_REAR_WHEEL] = 0;
  joint_efforts[FORK_LIFT] = 0;
  joint_efforts[LEFT_ARM_LINK] = 0;
  joint_efforts[RIGHT_ARM_LINK] = 0;

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

void init_motors() {
  printf("Init Motors...\n");
  //init motor position
  left_motor.set_position(0);
  right_motor.set_position(0);
  lift_motor.set_position(0);
}

void init_sensors() {
  printf("Init Sensors...\n");
  //gyro_sensor.rate();
  //gyro_sensor.g_a();
  ir_sensor.proximity();
  color_sensor.color();
}

void init_node(char* argv) {
  printf("Init Node...\n");

  nh.initNode(argv);
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
    std::cout << "Usage: ./maskor_ev3_bobb3e_node <ip-of-ros-master>" << std::endl;
  }

  init_motors();
  init_sensors();

  usleep(1000000);

  printf("Entering loop....\n");

  while(1)
    {

      printf("Lift motor position: %d" , lift_motor.position());
      printf("Lift motor speed: %d" , lift_motor.speed());
      
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
