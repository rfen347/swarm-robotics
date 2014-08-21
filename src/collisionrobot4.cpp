#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include "math.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

// TO-DO:
// - Make visitor walk around without bumping into things.

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage.
	px = 5 + msg.pose.pose.position.x;
	py =10 + msg.pose.pose.position.y;
	//ROS_INFO("Current x position is: %f", px);
	//ROS_INFO("Current y position is: %f", py);
}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}


void collisionCallback(geometry_msgs::Pose2D msg)
{

	ROS_INFO("Received location for robot 5 at %lf %lf", msg.x, msg.y);
	
	

}

int main(int argc, char **argv)
{
	
	//initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = -M_PI/2.0;
	px = 12.0;
	py = 3.0;
	
	//Initial velocity
	linear_x = 1;
	angular_z = 0;
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "RobotNode6");
	
	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;
	
	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_6/cmd_vel",1000);
	
	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_6/odom",1000, StageOdom_callback);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_6/base_scan",1000,StageLaser_callback);
	
	ros::Subscriber Robot_coord_sub = n.subscribe<geometry_msgs::Pose2D>("robot_5/pos",1000, collisionCallback);

	ros::Publisher Robot_coord_pub = n.advertise<geometry_msgs::Pose2D>("robot_6/pos",1000);


	ros::Rate loop_rate(10);
	
	//a count of howmany messages we have sent
	int count = 0;
	
	////messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;
	geometry_msgs::Pose2D Robot_pos;
	
	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
		
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);

		//location message
		Robot_pos.x = px;
		Robot_pos.y = py;
		Robot_pos.theta = theta;
		
		//publish the robot positions 
		Robot_coord_pub.publish(Robot_pos);
		
		ros::spinOnce();
		
		//ROS_INFO("Cycle %i - Visitor co-ordinates - (%f, %f)",count,px,py);
		
		loop_rate.sleep();
	}
	
	return 0;
	
}
