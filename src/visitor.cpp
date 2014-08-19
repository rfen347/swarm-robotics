#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
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

int main(int argc, char **argv)
{

 //initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = M_PI/2.0;
	px = 7;
	py = -4.5;
	
	//Initial velocity
	linear_x = 0;
	angular_z = 0;
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "RobotNode2");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_2/cmd_vel",1000); 

//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_2/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_2/base_scan",1000,StageLaser_callback);

ros::Rate loop_rate(10);

//a count of howmany messages we have sent
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

	ROS_INFO("Cycle %i - Visitor co-ordinates - (%f, %f)",count,px,py);

	loop_rate.sleep();
	++count;

	if(count==100){
		linear_x = 2;
	}
	if(count==110){
		angular_z =  M_PI / 2;
		linear_x = 0;
	} 
	if(count==120){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==155){
		angular_z = - M_PI / 2;;
		linear_x = 0;
	}
	if(count==165){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==235){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==245){
		angular_z = 0;
		linear_x = 2;
	}
	if(count > 265 && count < 640){
		ROS_INFO("ACTIVITY - Visitor watching TV");
		if (count % 2 < 1) {
			linear_x = -10;
		} else {linear_x = 10;}
	}
	if(count==640){
		angular_z = 0;
		linear_x = -2;
	}
	if(count==660){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==670){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==720){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	if(count==730){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==738){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==798){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==808){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	if(count==818){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==838){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==798){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==808){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	if(count==818){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==838){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==848){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==888){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	if(count==898){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==908){
		angular_z = 0;
		linear_x = 0;
	}
	if(count==928){
		angular_z = 0;
	}


}

return 0;

}
