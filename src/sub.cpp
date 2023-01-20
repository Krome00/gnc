#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cassert>
#include <sensor_msgs/LaserScan.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gnc_functions.hpp>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <math.h>
#include <geometry_msgs/TwistStamped.h>
#include <vector>

using namespace std;

std::ofstream csv_file;


void storing_data(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	geometry_msgs::Point current_pos;
	current_pos = get_current_location();

	csv_file << current_pos.x << "," << current_pos.y << std::endl;
	
}

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	geometry_msgs::Point current_pos;
	current_pos = get_current_location();
	float distance = sqrt(pow(current_pos.x, 2) + pow(current_pos.y, 2));
	// ROS_INFO("distance: %f", distance);
	// ROS_INFO("currentpos x: %f y: %f", current_pos.x, current_pos.y);
	for( int i=0; i < msg->bounding_boxes.size(); i++)
	{
		// detect = msg->bounding_boxes[i].Class;

		if (msg->bounding_boxes[i].Class == "person"){
			if(current_pos.x > -1 && current_pos.y < -1.1){
				ROS_INFO("human detected %f meters North-East within a 2.3 meter radius", distance);
			}else if(current_pos.y > 1 && current_pos.x > -1){
				ROS_INFO("human detected %f meters North-West within a 2.3 meter radius", distance);
			}else if( current_pos.x > 0 && -1 < current_pos.y < 1){
				ROS_INFO("human detected %f meters North within a 2.3 meter radius", distance);
			}
			// }else if(current_pos.x > 0 && current_pos.y < 0){
			// 	ROS_INFO("human detected %f meters South-West within a 2.3 meter radius", distance);
			// }else if(current_pos.x < 0 && current_pos.y < 0){
			// 	ROS_INFO("human detected %f meters South-East within a 2.3 meter radius", distance);
			// }
			// ROS_INFO("curr x: %f", current_pos.x);
			// ROS_INFO("curr y: %f", current_pos.y);
		}

	}
}



int main(int argc, char **argv){

	ros::init(argc, argv, "detection_sub");
	ros::NodeHandle n;

	// create and open the .csv file
	csv_file.open("System Testing 9", std::ios::out | std::ios::trunc);

	ros::Subscriber bounding_boxes_sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);
	ros::Subscriber store_data = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, storing_data);
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);
	
	ros::Rate rate(1);

	while (ros::ok()) {

		ros::spinOnce();
		rate.sleep();
	}
	// close .csv file
	csv_file.close();
	return 0;
}