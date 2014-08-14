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

void cook() {

	//navigate to food storage (7.35, -0.55)
	linear_x = 2;
	angular_z= 0.5;

	//indicate getting items from food storage
	//navigate to fridge (8.6, 0.15)
	//indicate getting items from fridge
	//navigate to stovetop (8.6, 3.6)
	//indicate cooking
	//navigate to table (2.12, -1.44)
	//inform resident food is served!
	//navigate back to idle position (5.5, 4.5)

}

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = 0 + msg.pose.pose.position.x;
	py = 0 + msg.pose.pose.position.y;
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
	px = 5.5;
	py = 4.5;
	
	//Initial velocity
	linear_x = 0;
	angular_z = 0;
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "RobotNode1");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_1/cmd_vel",1000); 

//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_1/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_1/base_scan",1000,StageLaser_callback);

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

	loop_rate.sleep();
	++count;

	ROS_INFO("Cycle %i - Cooking robot co-ordinates - (%f, %f)",count,px,py);

	//Start cooking at 5 seconds (testing)
	//if (count==50) {
	//	cook();
	//}

	if(count==200){
		linear_x = 2;
	}
	if(count==250){
		angular_z = M_PI / 2;
		linear_x = 0;
	} 
	if(count==260){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==280){
		ROS_INFO("ACTIVITY - Cooking robot starts taking food");
		angular_z = M_PI / 2;;
		linear_x = 0;
	}
	if(count==330){
		angular_z = 0;
	}
	if(count==340){
		linear_x = 2;
	}
	if(count==380){
		ROS_INFO("ACTIVITY - Cooking robot starts cooking");
		angular_z = - M_PI / 2;
		linear_x = 0;
	} 
	if(count==480){
		angular_z = 0;
	}


	if(count==500){
		linear_x = 2;
	}
	if(count==540){
		angular_z = -M_PI / 2;
		linear_x = 0;
	} 
	if(count==550){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==600){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	if(count==610){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==615){
		linear_x = 0;
		angular_z = M_PI / 2;
	}
	if(count==625){
		angular_z = 0;
		linear_x = 2;
	} 
	if(count==640){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}
	if(count==700){
		angular_z = 0;
	}



	if(count==720){
		linear_x = 2;
	}
	if(count==750){
		angular_z = -M_PI / 2;
		linear_x = 0;
	} 
	if(count==760){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==765){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==775){
		angular_z = 0;
		linear_x = 2;
	}
	if(count==790){
		angular_z = M_PI / 2;
		linear_x = 0;
	}
	if(count==800){
		angular_z = 0;
		linear_x = 2;
	} 
	if(count==850){
		angular_z = -M_PI / 2;
		linear_x = 0;
	}

	if(count==870){
		angular_z = 0;
	} 
	
}

return 0;

}

