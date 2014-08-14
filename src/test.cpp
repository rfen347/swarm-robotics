#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

#include <sstream>
#include "math.h"

#include <gtest/gtest.h>
#include "resident.cpp"

Resident r;
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


TEST(eatTest, residentTestCase6){
	ASSERT_EQ(2, angular_z);
}


TEST(getReadyToEatTest, residentTestCase7){

}

TEST(residentPositionTest, residentTestCase5){
	if (r.count==10){
		ASSERT_EQ(2, linear_x);
	}

	if (r.count==50){
		ASSERT_EQ(0, angular_z);
		ASSERT_EQ(2, linear_x);
	}
	
	if(count > 210 && count < 640){
			if (count % 2 < 1) {
				ASSERT_EQ(-10, linear_x);
			} else {
				ASSERT_EQ(10, linear_x);
			}
		}
}




//=================Visitor test cases============================================
Visitor v;
TEST(initialPositionTest, visitorTestCase1){
		ASSERT_EQ(0,linear_x);
		ASSERT_EQ(0, angular_z);
}

TEST(initialPoseTest, visitorTestCase2){
		ASSERT_EQ(M_PI/2.0, theta);
		ASSERT_EQ(7, px);
		ASSERT_EQ(-4.5, py);
}
TEST(visitorPositionTest, visitorTestCase3){
	if (r.count==100){
		ASSERT_EQ(2, linear_x);
	}

	if (r.count==165){
		ASSERT_EQ(0, angular_z);
		ASSERT_EQ(2, linear_x);
	}

	if(count > 265 && count < 640){
			if (count % 2 < 1) {
				ASSERT_EQ(-10, linear_x);
			} else {
				ASSERT_EQ(10, linear_x);
			}
		}
}


//=================Cooking robot test cases======================================

Cookingrobot c;
TEST(initialPositionTest, cookingrobotTestCase1){
		ASSERT_EQ(0,linear_x);
		ASSERT_EQ(0, angular_z);
}

TEST(initialPoseTest, cookingrobotTestCase2){
		ASSERT_EQ(M_PI/2.0, theta);
		ASSERT_EQ(5.5, px);
		ASSERT_EQ(4.5, py);
}
TEST(visitorPositionTest, cookingrobotTestCase3){
	if (r.count==200){
		ASSERT_EQ(2, linear_x);
	}

	if (r.count==260){
		ASSERT_EQ(0, angular_z);
		ASSERT_EQ(2, linear_x);
	}

	if (r.count==380){
			ASSERT_EQ(M_PI/2, angular_z);
			ASSERT_EQ(0, linear_x);
		}
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}

