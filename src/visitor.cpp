#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include "math.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

int loopRate = 10;

void setOrientation(){
	//Calculate the new value of theta
	theta = theta + (angular_z/loopRate);
	//Check for overflow
	if(theta>M_PI){ 
		theta = (theta-(M_PI*2));
	}else if(theta<=(M_PI*-1)){
		theta = theta + (M_PI*2);
	}
}



// TO-DO:
// - Make visitor walk around without bumping into things.


void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = 7.0 + msg.pose.pose.position.x;
	py =-4.5 + msg.pose.pose.position.y;

	//ROS_INFO("Current x position is: %f", px);
	//ROS_INFO("Current y position is: %f", py);

}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}


void rotateToAngle(double angle){


	//Calculate the angle to rotate
	double difference = theta - angle;
	//Don't rotate if we are at the correct angle
	if (difference == 0.0){
		return;
	}


	//Check for overflow
	if(difference>M_PI){ 
		difference = (difference-(M_PI*2));
	}else if(difference<(M_PI*-1)){
		difference = difference + (M_PI*2);
	}

	// Infrastructure
	ros::Rate loop_rate(loopRate);
	ros::NodeHandle n;
	geometry_msgs::Twist RobotNode_cmdvel;
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_2/cmd_vel",1000); 

	
	//Calculate the shortest angle velocity to rotate
	if(difference>0){
		angular_z = -M_PI/2;
		
	}else{
		angular_z = M_PI/2;
		
	}



		linear_x = 0;
	//Rotate to the specified angle
	while(theta!=angle){

		// Infrastructure
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		setOrientation();
		ros::spinOnce();
		loop_rate.sleep();
	}
	// ROS_INFO("I've stopped rotating. Theta is %f",theta * 180 / M_PI);
	angular_z = 0;
	linear_x = 2;

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

ros::Rate loop_rate(loopRate);

//a count of howmany messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;






while (ros::ok())
{

	//TO TEST rotateToAngle
	if(count == 60){
		rotateToAngle(0);
	
		rotateToAngle(-M_PI/2);
		rotateToAngle(M_PI);
		rotateToAngle(M_PI/2);

		rotateToAngle(M_PI/2);
		rotateToAngle(M_PI);
		rotateToAngle(-M_PI/2);
		rotateToAngle(0);
	}


	// linear_x = 10;
	// rotateToAngle(M_PI/-2);

	// rotateToAngle(M_PI);
	// rotateToAngle(M_PI/2);



	setOrientation();

	//messages to stage
	RobotNode_cmdvel.linear.x = linear_x;
	RobotNode_cmdvel.angular.z = angular_z;
        
	//publish the message
	RobotNode_stage_pub.publish(RobotNode_cmdvel);
	
	ros::spinOnce();

	//ROS_INFO("Cycle %i - Visitor co-ordinates - (%f, %f)",count,px,py);

	loop_rate.sleep();
	++count;
	
	// The old time-dependent navigation:
/*
	// enters house
	// go straight
	if(count==100){
		ROS_INFO("ACTIVITY - Visitor enters house");
		linear_x = 2;
	}
	//turn 90 degrees anti-clockwise
	if(count==110){
		angular_z =  M_PI / 2;
		linear_x = 0;
	} 
	// go straight
	if(count==120){
		angular_z = 0;
		linear_x = 2;
	}
	// turn 90 degrees clockwise
	if(count==155){
		angular_z = - M_PI / 2;;
		linear_x = 0;
	}
	// go straight
	if(count==165){
		angular_z = 0;
		linear_x = 2;
	}
	// turn 90 degrees anti-clockwise
	if(count==235){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	// go straight
	if(count==245){
		angular_z = 0;
		linear_x = 2;
	}
	// start wacthing TV (vibrate)
	if(count > 265 && count < 640){
		if (count == 266) {
			ROS_INFO("ACTIVITY - Visitor is watching TV");
		}
		if (count == 639) {
			ROS_INFO("ACTIVITY - Visitor has finished watching TV");
		}
		if (count % 2 < 1) {
			linear_x = -10;
		} else {linear_x = 10;}
	}
	// start walking to the dining table
	// go straight 
	if(count==640){
		angular_z = 0;
		linear_x = -2;
	}
	// turn 90 degrees anti-clockwise
	if(count==660){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	// go straight
	if(count==670){
		angular_z = 0;
		linear_x = 2;
	}
	//turn 90 degrees clockwise
	if(count==720){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	// go straight
	if(count==730){
		angular_z = 0;
		linear_x = 2;
	}
	// start eating
	// turn 90 degrees anti-clockwise
	if(count==738){
		ROS_INFO("ACTIVITY - Visitor is eating");
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	// finish eating and strat walking to the front door(leaving)
	// go straight
	if(count==798){
		ROS_INFO("ACTIVITY - Visitor has finished eating");
		angular_z = 0;
		linear_x = 2;
	}
	// turn 90 degrees clockwise
	if(count==808){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	// go straight
	if(count==818){
		angular_z = 0;
		linear_x = 2;
	}
	// turn 90 degrees anti-clockwise
	if(count==838){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	// go straight
	if(count==848){
		angular_z = 0;
		linear_x = 2;
	}
	// turn 90 degrees clockwise
	if(count==882){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	// go straight
	if(count==892){
		angular_z = 0;
		linear_x = 2;
	}
	// turn 90 degrees clockwise (back to its original facing direction)
	if(count==902){
		ROS_INFO("ACTIVITY - Visitor leaves house");
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	// stop
	if(count==922){
		angular_z = 0;
	}
	// reset the day
	if(count==1050){
		count = 0;
	}*/

}

return 0;

}
