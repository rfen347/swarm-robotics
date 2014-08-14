#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"

#include <gtest/gtest.h>
#include "resident.cpp"

Reisident r;
//when they fail, ASSERT_* yields a fatal failure and returns from the current function, 
//while EXPECT_* yields a nonfatal failure, allowing the function to continue running
//Normally, EXPECT_* is the better option since the rest of the test can continue to run and can give useful output.
//However, ASSERT_* is better if the test shouldn't continue.
//=================Resident test cases==========================================
TEST(initialPositionTest, residentTestCase1){
	ASSERT_EQ(0,linear_x);
	ASSERT_EQ(0, angular_z);
}

TEST(initialPoseTest, residentTestCase1){
	ASSERT_EQ(0, theta);
	ASSERT_EQ(-6.5, px);
	ASSERT_EQ(4.5, py);	
}

TEST(wakeUpTest, residentTestCase3){
	r.wakeUp();
	ASSERT_EQ(2, linear_x);	

}

TEST(stopEatingTest, residentTestCase4){
 	r.stopEating();
	ASSERT_EQ(0, angular_z);
}

TEST(residentPositionTest, residentTestCase5){
	
}
//=================Visitor test cases============================================

//=================Cooking robot test cases======================================
// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}








//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

void wakeUp()
{
	// Triggered by schedule.
	// Navigate to sofa, and then stop.
}

void getReadyToEat()
{
	// Triggered by robot message.
	// Navigate to dining table, and then stop.
}

void eat()
{
	// Spin on the spot to show that resident is eating. 
	angular_z = 2;	
}

void stopEating()
{
	angular_z=0;
}

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = 0 + msg.pose.pose.position.x;
	py =0 + msg.pose.pose.position.y;
	ROS_INFO("Current x position is: %f", px);
	ROS_INFO("Current y position is: %f", py);
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
	theta = M_PI/2.0;
	px = 10;
	py = 20;
	
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

	loop_rate.sleep();
	++count;

	if(count==80){
		eat();
	}else if(count==100){
		stopEating();
	}
}

return 0;

}
