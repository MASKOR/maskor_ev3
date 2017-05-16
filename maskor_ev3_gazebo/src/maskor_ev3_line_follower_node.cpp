#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

class LineFollower {
public:
  LineFollower();

private:
  void lineFollowerCallback(const std_msgs::Int8::ConstPtr& msg);

  

  ros::Subscriber sub;
  ros::Publisher vel_pub_;
  ros::NodeHandle n;
};

LineFollower::LineFollower(){

  sub = n.subscribe<std_msgs::Int8>("chatter",1000, &LineFollower::lineFollowerCallback,this);
  //sub = n.subscribe("chatter", 1000, LineFollower::lineFollowerCallback);

  vel_pub_ = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

}

void LineFollower::lineFollowerCallback(const std_msgs::Int8::ConstPtr& msg)
{
  ROS_INFO("I heard: [%i]", msg->data);
  geometry_msgs::Twist twist;
  bool trackOk = true;

  if(msg->data != 1){
    trackOk = false;
  }



    if(trackOk){
      twist.linear.x = 2;
      twist.angular.z = 0;
      vel_pub_.publish(twist);
    }
    else{
      twist.linear.x = 0;
      twist.angular.z = 2;
      vel_pub_.publish(twist);


    }


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lineFollower");
  LineFollower lineFollower;

  while(ros::ok()){
    ros::spin();
  }

  return 0;
}
