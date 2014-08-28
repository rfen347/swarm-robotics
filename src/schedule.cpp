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
#include <string>

using namespace std;
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
	//command ill
	string command="";

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

	ros::Publisher resident_ill = n.advertise<std_msgs::String>("robot_0/ill",1000);

	//subscribe to listen to messages coming from stage
	ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_3/odom",1000, StageOdom_callback);
	ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_3/base_scan",1000,StageLaser_callback);

	// advertise topics to other nodes

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

	// to CookingRobot R1
	ros::Publisher robot_cooking = n.advertise<project1::move>("robot_1/cooking",1000);

	// to Visitor R2
	ros::Publisher visitor_visit = n.advertise<project1::move>("robot_2/visit",1000);

	// to MedicalRobot R4
	ros::Publisher robot_giveMedication = n.advertise<project1::move>("robot_4/giveMedication",1000);
	ros::Publisher robot_callDoctor = n.advertise<project1::move>("robot_4/callDoctor",1000);

	// to EntertainmentRobot R5
	ros::Publisher robot_giveEntertainment = n.advertise<project1::move>("robot_5/giveEntertainment",1000);

	// to CompainionRobot R6
	ros::Publisher robot_giveCompanionship = n.advertise<project1::move>("robot_6/giveCompanionship",1000);

	// to Caregiver R7
	ros::Publisher caregiver_helpShower = n.advertise<project1::move>("robot_7/helpShower",1000);
	ros::Publisher caregiver_helpEat = n.advertise<project1::move>("robot_7/helpEat",1000);
	ros::Publisher caregiver_helpExercise = n.advertise<project1::move>("robot_7/helpExercise",1000);
	ros::Publisher caregiver_giveMoralSupport = n.advertise<project1::move>("robot_7/giveMoralSupport",1000);

	// to Relatives R8
	ros::Publisher relative_visit = n.advertise<project1::move>("robot_8/visit",1000);

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

	std_msgs::String msg;


	while (ros::ok())
	{
		//messages to stage
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
        
		//publish the message
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		if (count == 1){

			while (1 < 2){
				cin >>command;

				std::stringstream ss;

				if(command=="ill"){
					ss << "call Ill";
					msg.data = ss.str();
					resident_ill.publish(msg);
					break;
				}
				else if(command=="em"){
					ss << "emergency";
					msg.data = ss.str();
					resident_ill.publish(msg);
					break;
				}
				else if(command=="n"){
					ss << "normal day";
					msg.data = ss.str();
					resident_ill.publish(msg);
					break;
				} else {
					ROS_INFO("invalid input, please use either n, ill, or em");
				}
			}

		}
	
		if (command == "n"){
		if (count == 20){
			resident_wake.publish(Mo);
		}
		
		// resident uses toilet
		if (count == 55){
			resident_useToilet.publish(Mo);
		}
		
		// caregiver comes
		if (count == 150){
			caregiver_helpShower.publish(Mo);
		}

		// resident is using sink
		if (count == 300){
			resident_useSink.publish(Mo);
		}
		
		// resident goes to shower
		if (count == 430){
			resident_shower.publish(Mo);
		}
		

		// moving into lounge
		if (count == 530){
			resident_bathroomToLounge.publish(Mo);
			caregiver_helpExercise.publish(Mo);
		}

		// once in lounge, resident exercises
		if (count==690){
			resident_exercise.publish(Mo);
		}
	
		// cooking robot starts making food
		if (count==800){
			robot_cooking.publish(Mo);
		}

		// resident eats
		if (count==1250){
			robot_giveMedication.publish(Mo);
			resident_getReadyToEat.publish(Mo);
		}
	
		if(count==1350){
			resident_eat.publish(Mo);
			caregiver_helpEat.publish(Mo);
		}
		
		if(count==1400){
			resident_takeMedication.publish(Mo);
		}

		if(count==1460){
			resident_tableToSofa.publish(Mo);
			caregiver_giveMoralSupport.publish(Mo);
		}

		if(count==1600){
			resident_converseWithCaregiver.publish(Mo);
			robot_giveEntertainment.publish(Mo);
		}

		if(count==1650){
			visitor_visit.publish(Mo);
		}

		if(count==1700){
			relative_visit.publish(Mo);
		}

		if(count==2200){
			robot_giveCompanionship.publish(Mo);
		}

		if(count==2300){
			resident_goToBed.publish(Mo);
		}

		// another day starts
		if (count == 2550){
			count=0;
			ROS_INFO("DAY ENDS");
		} 
		}
		
		// emergency day
		if (command == "em"){

		if (count == 10) {
			robot_callDoctor.publish(Mo);
		}
		
		if (count== 50){
			doctor_visit.publish(Mo);	
		}

		if (count == 75){
			nurse_visit.publish(Mo);
					
		}
		if (count ==450){
			resident_takeMedication.publish(Mo);
		}
		
		if (count == 510){
			resident_goToAmbulance.publish(Mo);	
		} 
		
		if (count == 900){
			ROS_INFO("Resident is dead on his way to the hospitl, RIP");	
		} }

		// sick day
		if (command == "ill"){

		if (count == 10) {
			robot_callDoctor.publish(Mo);
		}

		if (count== 60){
			doctor_visit.publish(Mo);	
		}

		if (count == 75){
			nurse_visit.publish(Mo);
					
		}
		if (count == 450){
			resident_takeMedication.publish(Mo);
		}

		if (count == 900){
			count=0;
			ROS_INFO("DAY ENDS");
		}}

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;

}
