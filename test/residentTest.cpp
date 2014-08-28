#include <gtest/gtest.h>

#include "../src/resident.cpp"
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
	class residentTest : public testing::Test 
	{
	public:
		ros::NodeHandle n;
		resident r;

		residentTest() : n("~"), r(n,1){

		}

		virtual ~residentTest(){}

		virtual void SetUp(){}

		virtual void TearDown(){}
	
		/* data */
	};


	/**
	* Test if the resident node is created at the correct position
	* with the correct orientation and velocity.
	*/
	TEST_F(residentTest, testResidentInitial){
		ASSERT_EQ(r.px, -6.5);
		ASSERT_EQ(r.py, 4.5);
		ASSERT_EQ(r.theta, 0.0);
		ASSERT_EQ(r.linear_x, 0.0);
		ASSERT_EQ(r.angular_z, 0.0);
	}

	/**
	* Test if the resident can rotate to the appropriate angle,
	* (rotate down) 90 degrees clockwise
	*/
	TEST_F(residentTest, testRotate90Clockwise){
		r.rotateToAngle(-M_PI/2);
		ASSERT_EQ(r.theta,-M_PI/2);
	}

	/**
	* Test if the resident can rotate to the appropriate angle,
	* (rotate up) 180 degrees
	*/
	TEST_F(residentTest, testRotate180){
		r.rotateToAngle(M_PI/2);
		ASSERT_EQ(r.theta,M_PI/2);
	}

	/**
	* Test if the resident can rotate to the appropriate angle,
	* (rotate up) 90 degrees anticlockwise
	*/
	TEST_F(residentTest, testRotate90Anticlock){
		r.rotateToAngle(M_PI);
		ASSERT_EQ(r.theta,M_PI);
	}

	/**
	* Test if the resident can rotate to the appropriate angle,
	* (rotate up) 0 degrees
	*/
	TEST_F(residentTest, testRotate180){
		r.rotateToAngle(M_PI/2);
		ASSERT_EQ(r.theta,M_PI/2);
	}


	/**
	* Test that the navigate function works correctly
	* in the x direction, by moving the resident right by 
	* 2 units.
	*/
	TEST_F(residentTest, testNavigateRight){
		r.navigate(0,2.0);//Move 2 units to the right
		ASSERT_EQ(r.px, -4.5);
		ASSERT_EQ(r.py, 4.5);
	}


	/**
	* Test that the navigate function works correctly
	* in the y direction, by moving the resident down by 
	* 1 unit.
	*/
	TEST_F(residentTest, testNavigateDown){
		r.navigate(3, 1.0); //Move 1 unit down
		ASSERT_EQ(r.px, -4.5);
		ASSERT_EQ(r.py, 3.5);

	}

	/**
	* Test that the navigate function works correctly
	* in the x direction, by moving the resident left by 
	* 2 units.
	*/
	TEST_F(residentTest, testNavigateLeft){
		r.navigate(2, 2.0); //Move 2 units left
		ASSERT_EQ(r.px, -6.5);
		ASSERT_EQ(r.py, 3.5);

	}

	/**
	* Test that the navigate function works correctly
	* in the y direction, by moving the resident up by 
	* 1 unit.
	*/
	TEST_F(residentTest, testNavigateUp){
		r.navigate(1, 1.0); //Move 1 unit up
		ASSERT_EQ(r.px, -6.5);
		ASSERT_EQ(r.py, 4.5);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "RobotNode0");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

