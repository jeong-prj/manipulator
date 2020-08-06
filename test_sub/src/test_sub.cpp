
#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include <std_msgs/Int32.h>

#include "test_sub/Float32Multi.h"

#include <eigen3/Eigen/Eigen>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>


void PoseCallback(const test_sub::Float32Multi &test_msg){
	
	ROS_INFO("sub::%lf", test_msg.d[0]);
}


int main(int argc, char **argv){
    
	ros::init(argc, argv,"test_turtle_mani");
	ros::AsyncSpinner spinner(1); 
	spinner.start();
	
	
	if ( ! ros::master::check() )
		return false;
	
	//ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle nh("");

        ros::Subscriber sub_ = nh.subscribe("/rvecs_msg", 10, PoseCallback);

	//ros::start();
	while (ros::ok())
	{
		ros::spinOnce();
	}
	
}
