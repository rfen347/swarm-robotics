#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <project1/move.h>
#include <sstream>
#include "math.h"
#include <time.h>

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = -12.0 + msg.pose.pose.position.x;
	py = 0 + msg.pose.pose.position.y;
}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number	
}

void rmovecallback(project1::move mo)
{
	ROS_INFO("%f", mo.x);
}

int main(int argc, char **argv)
{

	 //initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = 0;
	px = -12.0;
	py = 0;
	
	//Initial velocity
	linear_x = 0;
	angular_z = 0;
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "RobotNode3");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_3/cmd_vel",1000);

	ros::Publisher resident_move = n.advertise<std_msgs::String>("robot_0/bbb",1000);
	ros::Publisher caregiver_move = n.advertise<std_msgs::String>("robot_7/bbb",1000);

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_3/odom",1000, StageOdom_callback);
	ros::Subscriber lrmo = n.subscribe<project1::move>("robot_0/rmove",1000, rmovecallback);

	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_3/base_scan",1000,StageLaser_callback);

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	//messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	// move
	//project1::move Mo;
	std_msgs::String Mo;

	std::stringstream ss;
	ss << "wake up";
	Mo.data = ss.str();

	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
        
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		if ( count == 20 ){

			resident_move.publish(Mo);
			caregiver_move.publish(Mo);
		}

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}
