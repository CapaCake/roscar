#include "ros/ros.h"
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

void Callback(const nav_msgs::Odometry::ConstPtr &odomMsg)
{
    
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"stop");		//创建节点 节点名 
	ros::NodeHandle nh;	
	ros::Subscriber sub = n.subscribe("encoder_imu_mix", 100, Callback)；
}


        



