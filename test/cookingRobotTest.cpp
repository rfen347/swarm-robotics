#include <gtest/gtest.h>

#include "../src/cookingrobot.cpp"
#include <project1/move.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"


using namespace project1;
namespace {
	class cookingRobotTest : public testing::Test 
	{
	public:
		ros::NodeHandle n;
		cookingrobot cr;

		cookingRobotTest() : n("~"), cr(n,1){

		}

		virtual ~cookingRobotTest(){}

		virtual void SetUp(){}

		virtual void TearDown(){}
	
		/* data */
	};


	/**
	* Test if the cooking robot node is created at the correct position
	* with the correct orientation and velocity.
	*/
	TEST_F(cookingRobotTest, testCookingRobotInitial){
		ASSERT_EQ(r.px, 5.5);
		ASSERT_EQ(r.py, 4.5);
		ASSERT_EQ(r.theta, 0.0);
		ASSERT_EQ(r.linear_x, 0.0);
		ASSERT_EQ(r.angular_z, 0.0);
	}

	/**
	* Test if the cooking robot can rotate to the appropriate angle,
	* (rotate down) 90 degrees clockwise
	*/
	TEST_F(cookingRobotTest, testRotate90Clockwise){
		r.rotateToAngle(-M_PI/2);
		ASSERT_EQ(r.theta,-M_PI/2);
	}

	/**
	* Test if the cooking robot can rotate to the appropriate angle,
	* (rotate up) 180 degrees
	*/
	TEST_F(cookingRobotTest, testRotate180){
		r.rotateToAngle(M_PI/2);
		ASSERT_EQ(r.theta,M_PI/2);
	}

	/**
	* Test if the cooking robot can rotate to the appropriate angle,
	* (rotate up) 90 degrees anticlockwise
	*/
	TEST_F(cookingRobotTest, testRotate90Anticlock){
		r.rotateToAngle(M_PI);
		ASSERT_EQ(r.theta,M_PI);
	}

	/**
	* Test if the cooking robot can rotate to the appropriate angle,
	* (rotate up) 0 degrees
	*/
	TEST_F(cookingRobotTest, testRotate180){
		r.rotateToAngle(M_PI/2);
		ASSERT_EQ(r.theta,M_PI/2);
	}

	/**
	* Test that the navigate function works correctly
	* in the y direction, by moving the cooking robot down by 
	* 3 units.
	*/
	TEST_F(cookingRobotTest, testNavigateDown){
		r.navigate(3, 3.0); //Move 3 unit down
		ASSERT_EQ(r.px, 5.5);
		ASSERT_EQ(r.py, 1.5);

	}


	/**
	* Test that the navigate function works correctly
	* in the x direction, by moving the cooking robot right by 
	* 1 unit.
	*/
	TEST_F(cookingRobotTest, testNavigateRight){
		r.navigate(0,1.0);//Move 1 unit to the right
		ASSERT_EQ(r.px, 6.5);
		ASSERT_EQ(r.py, 1.5);
	}



	/**
	* Test that the navigate function works correctly
	* in the x direction, by moving the cooking robot left by 
	* 1 unit.
	*/
	TEST_F(cookingRobotTest, testNavigateLeft){
		r.navigate(2, 1.0); //Move 1 unit left
		ASSERT_EQ(r.px, 5.5);
		ASSERT_EQ(r.py, 1.5);

	}

	/**
	* Test that the navigate function works correctly
	* in the y direction, by moving the cooking robot up by 
	* 3 units.
	*/
	TEST_F(cookingRobotTest, testNavigateUp){
		r.navigate(1, 3.0); //Move 3 units up
		ASSERT_EQ(r.px, 5.5);
		ASSERT_EQ(r.py, 4.5);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "RobotNode1");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

