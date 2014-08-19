#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include "math.h"
//#include "cookingrobot.h"


//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

void StageLaser_callback(sensor_msgs::LaserScan msg);
void StageOdom_callback(nav_msgs::Odometry msg);

void move(){
	linear_x=2;
}

void stopMove(){
	linear_x=0;
}

void spin(){
	angular_z=2;
}

void stopSpin(){
	angular_z=0;
}

void wakeUp()
{
	// Triggered by schedule.
	// Navigate to sofa, and then stop.
	// Navigate from (-6.5, 4.5) to (-3.5, 4.5), which is 3 units East.
	move();
	while(true){
		if(px>-3){
			stopMove();
			return 0;
		}
	}
}

void getReadyToEat()
{
	// Triggered by robot message.
	// Navigate from bedroom to dining table, and then stop.
	// Navigate from (-3.5, 4.5) to (-3.5,-1.0), which is 5.5 units South.
	// Navigate from (-3.5, -1.0) to (-0.5, -1.0), which is 3 units East.
}


void eat()
{
	spin();
}

void stopEating()
{
	// Stop spinning to show that the resident has stopped eating.
	stopSpin();
}

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = -6.5 + msg.pose.pose.position.x;
	py = 4.5 + msg.pose.pose.position.y;
}

void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

int main(int argc, char **argv)
{
// Create a schedule object

 //initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = 0;
	px = -6.5;
	py = 4.5;
	
	//Initial velocity
	linear_x = 2;
	angular_z = 0;
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "RobotNode0");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);

ros::Rate loop_rate(10);

//a count of how many messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;

while (ros::ok())
{
	//messages to stage
	RobotNode_cmdvel.linear.x = linear_x;
	RobotNode_cmdvel.angular.z = angular_z;
        
	//publish the message
	RobotNode_stage_pub.publish(RobotNode_cmdvel);
	
	ros::spinOnce();

	//ROS_INFO("Cycle %i - Resident co-ordinates - (%f, %f)",count,px,py);

	loop_rate.sleep();
	++count;

	ROS_INFO("Cycle %i - Resident co-ordinates - (%f,%f)",count,px,py);

	wakeUp();
}

return 0;

}
