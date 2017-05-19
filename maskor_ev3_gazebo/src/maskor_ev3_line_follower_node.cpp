#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>

class LineFollower {
public:
  LineFollower();

private:
  void lineFollowerCallback(const std_msgs::Int8::ConstPtr& msg);

  bool lastblackr = false;
  bool lastblackl = false;
  bool drive[5] {false};
  bool tracknotOk = true;
  int count = 0;

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
  //ROS_INFO("I heard: [%i]", msg->data);
  geometry_msgs::Twist twist;
  bool find_whitel = false;
  bool find_whiter = false;
 //finde linke Seite
 if(lastblackl == false && lastblackr == false){
   drive[3] = true;
   if(msg->data == 0){
     lastblackr = true;

     find_whitel = true;
   }
 }
 else if(find_whitel){
   drive[0] =false;
   drive[1] =false;
   drive[2] =true;
   drive[3] =false;

   if(msg->data == 0){
     drive[0] =false;
     drive[1] =false;
     drive[2] =false;
     drive[3] =false;
     drive[4] =true;
     find_whitel = false;
   }
 }
 else{
   if(msg->data == 1){
     drive[0] =true;
     drive[1] =false;
     drive[2] =false;
     drive[3] =false;
     count = 0;
   }
   else{
     if(lastblackl){
      drive[0] = false;
      drive[1] = false;
      drive[2] = false;
      drive[3] = true;


      if(msg->data == 1){
        drive[0] =true;
        drive[1] =false;
        drive[2] =false;
        drive[3] =false;

        if(msg->data == 0){
          drive[0] =false;
          drive[1] =false;
          drive[2] =false;
          drive[3] =false;
          drive[4] =true;
          //find_whiter = false;

          lastblackr = false;
          lastblackl = true;

        }



      }
     }
     else if(lastblackr){
       drive[0] = false;
       drive[1] = false;
       drive[2] = true;
       drive[3] = false;

       if(msg->data == 1){
         drive[0] =true;
         drive[1] =false;
         drive[2] =false;
         drive[3] =false;

         if(msg->data == 0){
           drive[0] =false;
           drive[1] =false;
           drive[2] =false;
           drive[3] =false;
           drive[4] =true;

           lastblackr = true;
           lastblackl = false;
         }

       }
     }
   }
 }






  //VORWÄRTS
  if(drive[0]){
    std::cout << "vörwärts" << '\n';
    twist.linear.x = 2;
    twist.angular.z = 0;
    vel_pub_.publish(twist);
  }
  //RÜCKWÄRTS
  else if(drive[1]){
    std::cout << "rückwärts" << '\n';
    //lastblackr = false;
    twist.linear.x = -2;
    twist.angular.z = 0;
    vel_pub_.publish(twist);
  }
  //RECHTS
  else if(drive[2]){
    std::cout << "rechts" << '\n';
    //lastblackr = false;
    twist.linear.x = 0;
    twist.angular.z = -20;
    vel_pub_.publish(twist);
  }
  //LINKS
  else if(drive[3]){
    std::cout << "links" << '\n';
    twist.linear.x = 0;
    twist.angular.z = 20;
    vel_pub_.publish(twist);
  }
  //STOPP
  else if(drive[4]){
    std::cout << "stopp" << '\n';
    twist.linear.x = 0;
    twist.angular.z = 0;
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
