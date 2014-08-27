#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <rosgraph_msgs/Clock.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <project1/move.h>
#include <sstream>
#include "math.h"
#include <time.h>

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = -12.0 + msg.pose.pose.position.x;
	py = 0 + msg.pose.pose.position.y;
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
	theta = 0;
	px = -12.0;
	py = 0;
	
	//Initial velocity
	linear_x = 0;
	angular_z = 0;
	
	//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
	ros::init(argc, argv, "RobotNode3");

	//NodeHandle is the main access point to communicate with ros.
	ros::NodeHandle n;

	//advertise() function will tell ROS that you want to publish on a given topic_
	//to stage
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_3/cmd_vel",1000);

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_3/odom",1000, StageOdom_callback);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_3/base_scan",1000,StageLaser_callback);

	// advertise topics to other nodes
	// to CookingRobot R1
	ros::Publisher robot_cooking = n.advertise<project1::move>("robot_1/cooking",1000);

	// to CompainionRobot R6
	ros::Publisher robot_giveCompanionship = n.advertise<project1::move>("robot_6/giveCompanionship",1000);

	// to MedicalRobot R4
	ros::Publisher robot_giveMedication = n.advertise<project1::move>("robot_4/giveMedication",1000);
	ros::Publisher robot_callDoctor = n.advertise<project1::move>("robot_4/callDoctor",1000);

	// to EntertainmentRobot R5
	ros::Publisher robot_giveEntertainment = n.advertise<project1::move>("robot_5/giveEntertainment",1000);

	// to Resident R0
	ros::Publisher resident_wake = n.advertise<project1::move>("robot_0/wake",1000);
	ros::Publisher resident_useToilet = n.advertise<project1::move>("robot_0/useToilet",1000);
	ros::Publisher resident_useSink = n.advertise<project1::move>("robot_0/useSink",1000);
	ros::Publisher resident_shower = n.advertise<project1::move>("robot_0/shower",1000);
	ros::Publisher resident_bathroomToLounge = n.advertise<project1::move>("robot_0/bathroomToLounge",1000);
	ros::Publisher resident_exercise = n.advertise<project1::move>("robot_0/exercise",1000);
	ros::Publisher resident_getReadyToEat = n.advertise<project1::move>("robot_0/getReadyToEat",1000);
	ros::Publisher resident_eat = n.advertise<project1::move>("robot_0/eat",1000);
	ros::Publisher resident_takeMedication = n.advertise<project1::move>("robot_0/takeMedication",1000);
	ros::Publisher resident_tableToSofa = n.advertise<project1::move>("robot_0/tableToSofa",1000);
	ros::Publisher resident_converseWithCaregiver = n.advertise<project1::move>("robot_0/converseWithCaregiver",1000);
	ros::Publisher resident_acceptEntertainment = n.advertise<project1::move>("robot_0/acceptEntertainment",1000);
	ros::Publisher resident_acceptCompanionship = n.advertise<project1::move>("robot_0/acceptCompanionship",1000);
	ros::Publisher resident_goToBed = n.advertise<project1::move>("robot_0/goToBed",1000);
	ros::Publisher resident_goToAmbulance = n.advertise<project1::move>("robot_0/goToAmbulance",1000);

	// to Visitor R2
	ros::Publisher visitor_visit = n.advertise<project1::move>("robot_2/visit",1000);

	// to Relatives R8
	ros::Publisher relative_visit = n.advertise<project1::move>("robot_8/visit",1000);

	// to Caregiver R7
	ros::Publisher caregiver_helpShower = n.advertise<project1::move>("robot_7/helpShower",1000);
	ros::Publisher caregiver_helpEat = n.advertise<project1::move>("robot_7/helpEat",1000);
	ros::Publisher caregiver_helpExercise = n.advertise<project1::move>("robot_7/helpExercise",1000);
	ros::Publisher caregiver_giveMoralSupport = n.advertise<project1::move>("robot_7/giveMoralSupport",1000);

	// to Doctor R9
	ros::Publisher doctor_visit = n.advertise<project1::move>("robot_9/visit",1000);

	// to Nurse R10
	ros::Publisher nurse_visit = n.advertise<project1::move>("robot_10/visit",1000);

	ros::Rate loop_rate(10);

	//a count of howmany messages we have sent
	int count = 0;

	//messages
	//velocity of this RobotNode
	geometry_msgs::Twist RobotNode_cmdvel;

	// move
	project1::move Mo;

	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
        
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);

		if ( count == 5 ){

			resident_wake.publish(Mo);

		}

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}
