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

void StageLaser_callback(sensor_msgs::LaserScan msg);
void StageOdom_callback(nav_msgs::Odometry msg);

void wakeUp()
{
	// Triggered by schedule.
	// Navigate to sofa, and then stop.
	linear_x = 2;
	
	// Navigate from (-6.8,4.5) to (-3.5, 4.5), which is 3.3 units East.
}

void getReadyToEat()
{
	// Triggered by robot message.
	// Navigate to dining table, and then stop.
}


void stopEating()
{
	angular_z=0;
}

void eat()
{
	// Triggered by robot call.
	// Spin on the spot to show that resident is eating. 
	angular_z = 2;
	// Then after two seconds, it stops eating.

	// Triggered by robot call.
	// Navigate to dining table, and then stop.
	// Navigate from (-3.5, 4.5) to (-3.5,-1.0), which is 5.5 units South.
	// Navigate from (-3.5, -1.0) to (-0.5, -1.0), which is 3 units East.


}
void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = 0 + msg.pose.pose.position.x;
	py =0 + msg.pose.pose.position.y;
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
// Create a schedule object

 //initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = 0;
	px = -6.5;
	py = 4.5;
	
	//Initial velocity
	linear_x = 0;
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

	ROS_INFO("Cycle %i - Resident co-ordinates - (%f, %f)",count,px,py);

	loop_rate.sleep();
	++count;

	if(count==10){
		linear_x = 2;
	}
	if(count==40){
		angular_z = - M_PI / 2;
		linear_x = 0;
	} 
	if(count==50){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==100){
		angular_z = M_PI / 2;;
		linear_x = 0;
	}
	if(count==110){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==140){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==150){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==190){
		angular_z = - M_PI / 2;
		linear_x = 0;
	}
	if(count==200){
		angular_z = 0;
		linear_x = 2;
	}
	if(count > 210 && count < 640){
		ROS_INFO("ACTIVITY - Resident is watching TV");
		if (count % 2 < 1) {
			linear_x = -10;
		} else {linear_x = 10;}
	}
	if(count==640){
		angular_z = 0;
		linear_x = -2;
	}
	if(count==660){
		angular_z = - M_PI / 2;
		linear_x = 0;
	}
	if(count==670){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==720){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==730){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==735){
		angular_z = - M_PI / 2;
		linear_x = 0;
	}
	if(count==795){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==820){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	if(count==830){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==890){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==900){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==930){
		angular_z = - M_PI / 2;
		linear_x = 0;
	}

	if(count==950){
		angular_z = 0;
	}
	

	

}

return 0;

}
