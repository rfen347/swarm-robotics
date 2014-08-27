#include <gtest/gtest.h>

#include "../src/visitor.cpp"
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
	class visitorTest : public testing::Test 
	{
	public:
		ros::NodeHandle n;
		visitor v;

		visitorTest() : n("~"), v(n,1){

		}

		virtual ~visitorTest(){}

		virtual void SetUp(){}

		virtual void TearDown(){}
	
		/* data */
	};


	/**
	* Test if the visitor node is created at the correct position
	* with the correct orientation and velocity.
	*/
	TEST_F(visitorTest, testVisitorInitial){
		ASSERT_EQ(r.px, -6.5);
		ASSERT_EQ(r.py, 4.5);
		ASSERT_EQ(r.theta, 0.0);
		ASSERT_EQ(r.linear_x, 0.0);
		ASSERT_EQ(r.angular_z, 0.0);
	}

	/**
	* Test if the visitor can rotate to the appropriate angle,
	* (rotate down) 90 degrees clockwise
	*/
	TEST_F(visitorTest, testRotate90Clockwise){
		r.rotateToAngle(-M_PI/2);
		ASSERT_EQ(r.theta,-M_PI/2);
	}

	/**
	* Test if the visitor can rotate to the appropriate angle,
	* (rotate up) 180 degrees
	*/
	TEST_F(visitorTest, testRotate180){
		r.rotateToAngle(M_PI/2);
		ASSERT_EQ(r.theta,M_PI/2);
	}

	/**
	* Test if the visitor can rotate to the appropriate angle,
	* (rotate up) 90 degrees anticlockwise
	*/
	TEST_F(visitorTest, testRotate90Anticlock){
		r.rotateToAngle(M_PI);
		ASSERT_EQ(r.theta,M_PI);
	}

	/**
	* Test if the visitor can rotate to the appropriate angle,
	* (rotate up) 0 degrees
	*/
	TEST_F(visitorTest, testRotate180){
		r.rotateToAngle(M_PI/2);
		ASSERT_EQ(r.theta,M_PI/2);
	}


	/**
	* Test that the navigate function works correctly
	* in the x direction, by moving the visitor right by 
	* 2 units.
	*/
	TEST_F(visitorTest, testNavigateRight){
		r.navigate(0,2.0);//Move 2 units to the right
		ASSERT_EQ(r.px, -4.5);
		ASSERT_EQ(r.py, 4.5);
	}


	/**
	* Test that the navigate function works correctly
	* in the y direction, by moving the visitor down by 
	* 1 unit.
	*/
	TEST_F(visitorTest, testNavigateDown){
		r.navigate(3, 1.0); //Move 1 unit down
		ASSERT_EQ(r.px, -4.5);
		ASSERT_EQ(r.py, 3.5);

	}

	/**
	* Test that the navigate function works correctly
	* in the x direction, by moving the visitor left by 
	* 2 units.
	*/
	TEST_F(visitorTest, testNavigateLeft){
		r.navigate(2, 2.0); //Move 2 units left
		ASSERT_EQ(r.px, -6.5);
		ASSERT_EQ(r.py, 3.5);

	}

	/**
	* Test that the navigate function works correctly
	* in the y direction, by moving the visitor up by 
	* 1 unit.
	*/
	TEST_F(visitorTest, testNavigateUp){
		r.navigate(1, 1.0); //Move 1 unit up
		ASSERT_EQ(r.px, -6.5);
		ASSERT_EQ(r.py, 4.5);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "RobotNode3");
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

