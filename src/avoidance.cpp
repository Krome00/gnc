#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <gnc_functions.hpp>
#include <mavros_msgs/WaypointSetCurrent.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandLong.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#define LINEAR_VEL 0.5 // m/s
#define ANGULAR_VEL 0.5 // rad/s

int counter = 0;
ros::Time start_time;
ros::Time end_time;

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg){	
	sensor_msgs::LaserScan current_2D_scan; // getting 2d lidar data
	geometry_msgs::Point current_pos; //getting current position
  	current_2D_scan = *msg; // getting 2D lidar data
  	// float target_psi[10] = {0, 90, 0, -90, -180, -90, 0, -90, -180, 90};
	// float target_x[3] = {0,10,10};
	// float target_y[3] = {0,10,10};
	float target_psi = 0;
	float attraction_vector_x = 0;
	float attraction_vector_y = 0;
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	float Ff_x; // x-coordinate of the free force
	float Ff_y; // y-coordinate of the free force
	float target_x[10] = { 0, -9, -9, -3, -3, 3, 3, 9, 9, 0 };
	float target_y[10] = { 0, 0, 15, 15, 5, 5, 15, 15, 0, 0 };
	float altitude = 4;
  	//getting current position
	current_pos = get_current_location();
	bool avoid = false;
	float zone_radius = 40; // The zone of attraction has a radius of 21 meter
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180);
	float fix = 0.75; //rate of fixing the destination

	// Compute distance between robot and target
	float distance = sqrt(pow(current_pos.x - target_x[counter], 2) + pow(current_pos.y - target_y[counter], 2));
	
	if(distance < 0.65 && current_pos.z > 3){
		counter++;
		sleep(1);
	}
	// Check if the drone has reached the final target position
	if(counter == 10){
		land();
	}
	if (distance < zone_radius){
		// If the robot is within the zone of attraction,
		// compute the attraction vector as a unit vector pointing towards the target
		attraction_vector_x = (target_x[counter] - current_pos.x) / distance;
		attraction_vector_y = (target_y[counter] - current_pos.y) / distance;
	}
	// Transform attraction vector to robot's frame of reference
	attraction_vector_x = attraction_vector_x*cos((current_heading)*deg2rad) - attraction_vector_y*sin((current_heading)*deg2rad);
	attraction_vector_y = attraction_vector_x*sin((current_heading)*deg2rad) + attraction_vector_y*cos((current_heading)*deg2rad);


	// Update destination based on attraction vector, also has condition loop to correct the increment of values to not let the drone be astray
	if(target_x[counter] >= current_pos.x && target_y[counter] >= current_pos.y){
		set_destination(attraction_vector_x + current_pos.x, attraction_vector_y + current_pos.y, altitude, target_psi);
	}else if(target_x[counter] < current_pos.x && target_y[counter] > current_pos.y){
		set_destination(current_pos.x - fix, attraction_vector_y + current_pos.y, altitude, target_psi);
	}else if(target_x[counter] > current_pos.x && target_y[counter] < current_pos.y){
		set_destination(attraction_vector_x + current_pos.x, current_pos.y - fix, altitude, target_psi);
	}else if(target_x[counter] < current_pos.x && target_y[counter] < current_pos.y){
		set_destination(current_pos.x - fix, current_pos.y - fix, altitude, target_psi);
	}


	

    for(int j=1; j<current_2D_scan.ranges.size(); j++){ //iterate lidar data

		float d0 = 2; //threshold or zone where the drone would consider avoidance 
		float k = 0.2;//positive scaling factor

		if(current_2D_scan.ranges[j] < d0 && current_2D_scan.ranges[j] > .35){ // if obstacle is between threshold and 0.35 or minimum distance between the drone and obstacle
			
			avoid = true;
			float x = cos(current_2D_scan.angle_increment*j);
			float y = sin(current_2D_scan.angle_increment*j);
			float U = -.5*k*pow(((1/current_2D_scan.ranges[j]) - (1/d0)), 2);	//magnitude of avoidance
			float Fcf = -0.5 * k * pow((1/current_2D_scan.ranges[j]) - (1/d0), 2); // Magnitude of the constant force
			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;

			// float Ff; // Magnitude of the free force
			float theta = current_heading * deg2rad; // Angle of the contact force
			float ex; // x-coordinate of the unit vector in the direction of the contact force
			float ey; // y-coordinate of the unit vector in the direction of the contact force
			// Calculate the x- and y-coordinates of the unit vector in the direction of the contact force
			ex = cos(theta);
			ey = sin(theta);

			// Calculate the magnitude of the free force
			// Ff = Fcf * (ex * cos(theta) + ey * sin(theta));
			Ff_x =  Fcf * cos(theta) * ex;
			Ff_y =  Fcf * sin(theta) * ey;
		}
	}
	// Transform avoidance vector to robot's frame of reference
	avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
	avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);

	// Transform free force vector to robot's frame of reference
	Ff_x = Ff_x*cos((current_heading)*deg2rad) - Ff_y*sin((current_heading)*deg2rad);
	Ff_y = Ff_x*sin((current_heading)*deg2rad) + Ff_y*cos((current_heading)*deg2rad);

	if(avoid){
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3){
			avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		}
		float f_total_x = attraction_vector_x + avoidance_vector_x + Ff_x + current_pos.x;
		float f_total_y = attraction_vector_y + avoidance_vector_y + Ff_y + current_pos.y;
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		set_destination(f_total_x, f_total_y, altitude, target_psi);	
	}
	ROS_INFO("%d", counter);
	ROS_INFO("%f", target_x[counter]);
	ROS_INFO("%f", target_y[counter]);
	ROS_INFO("distance: %f", distance);

}

int main(int argc, char **argv){
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle n;
	ros::Publisher setpoint_raw_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);
	// Set the linear and angular velocities
	geometry_msgs::Twist twist;
	twist.linear.x = LINEAR_VEL;
	twist.angular.z = ANGULAR_VEL;
	// Publish the velocity commands
	setpoint_raw_pub.publish(twist);
	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();


	//request takeoff
	takeoff(4);

	// set_destination(0,0,2,0);
	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);

	while(ros::ok()){
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}