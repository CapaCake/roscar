//
//racecar
//

#include "../include/racecar_driver.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Twist.h>

int i=0;
void TwistCallback(const geometry_msgs::Twist& twist)
{


    double angle;
    //ROS_INFO("x= %f", twist.linear.x);
    //ROS_INFO("z= %f", twist.angular.z);
    angle = 2500.0 - twist.angular.z * 2000.0 / 180.0;
    if(angle==30)
    {
    angle=90;
    }
    //ROS_INFO("angle= %d",uint16_t(angle));
    send_cmd(uint16_t(twist.linear.x),uint16_t(angle));

}

int main(int argc, char** argv)
{
    char data[] = "/dev/car";
    art_racecar_init(38400,data);
    ros::init(argc, argv, "racecar_driver");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/car/cmd_vel",1,TwistCallback);



    ros::spin();

}
