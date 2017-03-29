/*
 * rosserial_embeddedlinux subscriber example
 *
 * Prints a string sent on a subscribed ros topic.
 * The string can be sent with e.g.
 * $ rostopic pub chatter std_msgs/String -- "Hello Embedded Linux"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>

ros::NodeHandle  nh;
//char *rosSrvrIp = "192.168.192.42";
char *rosSrvrIp = "127.0.0.1";
nav_msgs::Odometry odom_msg;


void cmd_velCb(const geometry_msgs::Twist& twist_msg) {
       printf("Received cmd_vel message\n");
}


ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", cmd_velCb );
ros::Publisher odom_pub("odom", &odom_msg);

int main()
{
        nh.initNode(rosSrvrIp);
        nh.subscribe(cmd_vel_sub);
	nh.advertise(odom_pub);

        while(1) {
                  sleep(1);
                  nh.spinOnce();
		  odom_pub.publish(&odom_msg);
        }
}
