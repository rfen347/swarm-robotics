#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include "math.h"
#include <project1/move.h>

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

int loopRate = 10;
double posAllowance = 0.005;

void StageLaser_callback(sensor_msgs::LaserScan msg);
void StageOdom_callback(nav_msgs::Odometry msg);

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

// This function makes the robot move forward the way it is facing.
void move(){
	linear_x=2;
}

// This function makes the robot stop moving forward.
void stopMove(){
	linear_x = 0;
}

// This function makes the robot stop rotating.
void stopRotation(){
	angular_z=0;
}

// This function makes the robot rotate fast.
void rotateFast(){
	angular_z=M_PI/2;
}

// Spin for the number of cycles specified
void spin(int cycles){

	// Infrastructure
	ros::Rate loop_rate(loopRate);
	ros::NodeHandle n;
	geometry_msgs::Twist RobotNode_cmdvel;
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	linear_x = 0;
	rotateFast(); 			// start spinning
	int counter = 0;
		
	while (counter < cycles) {
		counter++;

		// Infrastructure
		RobotNode_cmdvel.linear.x = linear_x;
		RobotNode_cmdvel.angular.z = angular_z;
		RobotNode_stage_pub.publish(RobotNode_cmdvel);
		setOrientation();
		ros::spinOnce();
		loop_rate.sleep();
	}

	stopRotation(); // stop spinning
}

// This function makes the robot rotate to a specific angle. The input is the angle measured in radians, where 0 is East/right and positive values are anticlockwise.
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
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000);
	
	//Calculate the shortest angle velocity to rotate
	if(difference>0){
		angular_z = -M_PI/2;
		
	}else{
		angular_z = M_PI/2;
		
	}

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
	angular_z = 0;
}

// This function makes the robot move in an orthogonal direction (North, East, South or West). Input the direction to move and the distance to move by. The robot will rotate and carry out this movement.

// Integer codes for direction:
// 0 = East/right
// 1 = North/up
// 2 = West/left
// 3 = South/down

void navigate(int direction, double distance)
{
	double dest = 0;

	distance = distance - 0.2;

	// Infrastructure
	ros::Rate loop_rate(loopRate);
	ros::NodeHandle n;
	geometry_msgs::Twist RobotNode_cmdvel;
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_0/cmd_vel",1000); 

	if (direction==0){ // Move East/right.
		// Rotating to face East with rotateToAngle().
		rotateToAngle(0);

		// Determine the destination co-ordinates.
		dest = px + distance;

		move();

		while(px<dest){
			//Break at position that is close enough
			if((dest-px)<posAllowance){
				break;
			}
			// Infrastructure
			RobotNode_cmdvel.linear.x = linear_x;
			RobotNode_cmdvel.angular.z = angular_z;
			RobotNode_stage_pub.publish(RobotNode_cmdvel);
			ros::spinOnce();
			loop_rate.sleep();
		}

	}else if (direction==1){ // Move North/up.
		// Rotating to face North with rotateToAngle().
		rotateToAngle(M_PI/2);

		dest = py + distance;

		move();
		while(py<dest){
			//Break at position that is close enough
			if((dest-py)<posAllowance){
				break;
			}
			// Infrastructure
			RobotNode_cmdvel.linear.x = linear_x;
			RobotNode_cmdvel.angular.z = angular_z;
			RobotNode_stage_pub.publish(RobotNode_cmdvel);
			ros::spinOnce();
			loop_rate.sleep();
		}

	}else if (direction==2){ // Move West/left.
		// Rotating to face West with rotateToAngle().
		rotateToAngle(M_PI);
		
		dest = px - distance;

		move();
		while(px>dest){
			//Break at position that is close enough
			if((px-dest)<posAllowance){
				break;
			}

			// Infrastructure
			RobotNode_cmdvel.linear.x = linear_x;
			RobotNode_cmdvel.angular.z = angular_z;
			RobotNode_stage_pub.publish(RobotNode_cmdvel);
			ros::spinOnce();
			loop_rate.sleep();
		}

	}else{ // Move South/down.
		// Rotating to face South with rotateToAngle().
		rotateToAngle(-M_PI/2);
		
		dest = py - distance;

		move();
		while(py>dest){
			//Break at position that is close enough
			if((py-dest)<posAllowance){
				break;
			}

			// Infrastructure
			RobotNode_cmdvel.linear.x = linear_x;
			RobotNode_cmdvel.angular.z = angular_z;
			RobotNode_stage_pub.publish(RobotNode_cmdvel);
			ros::spinOnce();
			loop_rate.sleep();
		}
	}
	//Stop the robot's movement once at the destination
	stopMove();

	// Infrastructure
	RobotNode_cmdvel.linear.x = linear_x;
	RobotNode_cmdvel.angular.z = angular_z;
	RobotNode_stage_pub.publish(RobotNode_cmdvel);
	ros::spinOnce();
	loop_rate.sleep();

	//Spin again to ensure in correct position
	ros::spinOnce();
	loop_rate.sleep();
}


//Schedule to call for the resident to start the day
void wakeUp(){
	ROS_INFO("Resident wakes up");
	navigate(0,3);
}

//Schedule to call for the resident to use the toilet
void useToilet(){
	ROS_INFO("Resident goes to the toilet");
	navigate(3,5);
	navigate(2,5);
	navigate(3,2);
	navigate(0,2.2);
	spin(40);
}

//Schedule to call for the resident to use the sink
void useSink(){
	ROS_INFO("Resident goes to the sink");
	navigate(2,2.6);
	spin(40);
}

//Schedule to call for the resident to shower
void shower(){
	ROS_INFO("Resident goes to the shower");
	navigate(3,0.9);
	spin(40);
}

//Schedule to call for the resident to go to the lounge
void bathroomToLounge(){
	ROS_INFO("Resident leaves the bathroom and enters the lounge");
	navigate(1,0.9);
	navigate(0,0.5);
	navigate(1,2);
	navigate(0,5);
}

//Schedule to call for the resident to exercise
void exercise(){
	ROS_INFO("Resident exercises");	
	navigate(3,3);
	navigate(0,10.5);
	navigate(2,10.5);
	navigate(0,10.5);
	navigate(2,10.5);
}

//Schedule to call for the resident to go to the table
void getReadyToEat(){
	ROS_INFO("Resident goes to the table to eat");
	navigate(1,3);
	navigate(0,2.5);
}

// Schedule to call for the resident to eat
void eat(){
	ROS_INFO("Resident eats");
	spin(40);
}

// Schedule to call for the resident to take medication
void takeMedication(){
	ROS_INFO("Resident takes medication");
	navigate(0,0);
	spin(50);
}

// Schedule to call for the resident to go to the sofa
void tableToSofa(){
	ROS_INFO("Resident moves to the sofa");
	navigate(1,3.8);
	navigate(0,1.6);
	navigate(1,0);
}

// Schedule to call for the resident to talk to the caregiver
void converseWithCaregiver(){
	ROS_INFO("Resident converses with the caregiver");
	spin(590);
}

// Schedule to call for the resident to accept entertainment from the entertainment robot
void acceptEntertainment(){
	ROS_INFO("Resident accepts Entertainment");
	spin(80);
}

// Schedule to call for the resident to accept companionship robot
void acceptCompanionship(){
	ROS_INFO("Resident video chats/talks to companionship robot");
	spin(50);
}

// Schedule to call for the resident to sleep
void goToSleep(){
	//Set back to original orientation
	rotateToAngle(0);

	ROS_INFO("Resident is sleeping");
}


// Schedule to call for the resident to go to bed
void goToBed(){
	ROS_INFO("Resident goes to bed");
	navigate(2,1.6);
	navigate(3,4);
	navigate(2,2.5);
	navigate(1,5.2);
	navigate(2,3.1);
	goToSleep();
}


// Schedule to call for the resident to get sick
void getSick(){
	ROS_INFO("Resident is sick");
	// No movement needed
}

// Schedule to call for the resident to go to the ambulance and leave the house
void goToAmbulance(){
	ROS_INFO("Resident leaves: Too sick");
	navigate(0,3);
	navigate(3,8);
	navigate(0,10.5);
	navigate(3,7);
}


void StageOdom_callback(nav_msgs::Odometry msg)
{	
	px = -6.5 + msg.pose.pose.position.x;
	py = 4.5 + msg.pose.pose.position.y;
}

void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	
}


/*void illCallback(std_msgs::String msg){

	if (msg.data == "emergency"){

		ROS_INFO("There is emergency");
		bool emergency=true;
	}
	else if (msg.data == "call Ill"){

		ROS_INFO("resident gets ill");
		bool ill=true;
	}
	else if (msg.data == "normal day"){
		ROS_INFO("normal day starts");
	}
	
} */

void wake_callback(project1::move){
	wakeUp();
}

void useToilet_callback(project1::move){
	useToilet();
}

void useSink_callback(project1::move){
	useSink();
}

void shower_callback(project1::move){
	shower();
}

void bathroomToLounge_callback(project1::move){
	bathroomToLounge();
}

void exercise_callback(project1::move){
	exercise();
}

void getReadyToEat_callback(project1::move){
	getReadyToEat();
}


void eat_callback(project1::move){
	eat();
}

void takeMedication_callback(project1::move){
	takeMedication();
}

void tableToSofa_callback(project1::move){
	tableToSofa();
}

void converseWithCaregiver_callback(project1::move){
	converseWithCaregiver();
}

void acceptEntertainment_callback(project1::move){
	acceptEntertainment();
}

void acceptCompanionship_callback(project1::move){
	acceptCompanionship();
}

void goToBed_callback(project1::move){
	goToBed();
}

void goToAmbulance_callback(project1::move){
	goToAmbulance();
}

void goToSleep_callback(project1::move){
	goToSleep();
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
ros::Publisher coordPublisher= n.advertise<project1::move>("robot_0/coord",1000); 

//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);	

//subscribe to listen to messages coming from the schedule node
ros::Subscriber wake_sub = n.subscribe<project1::move>("robot_0/wake",1000, wake_callback);
ros::Subscriber useToilet_sub = n.subscribe<project1::move>("robot_0/useToilet",1000, useToilet_callback);
ros::Subscriber useSink_sub = n.subscribe<project1::move>("robot_0/useSink",1000, useSink_callback);
ros::Subscriber shower_sub = n.subscribe<project1::move>("robot_0/shower",1000, shower_callback);
ros::Subscriber bathroomToLounge_sub = n.subscribe<project1::move>("robot_0/bathroomToLounge",1000, bathroomToLounge_callback);
ros::Subscriber exercise_sub = n.subscribe<project1::move>("robot_0/exercise",1000, exercise_callback);
ros::Subscriber getReadyToEat_sub = n.subscribe<project1::move>("robot_0/getReadyToEat",1000, getReadyToEat_callback);
ros::Subscriber eat_sub = n.subscribe<project1::move>("robot_0/eat",1000, eat_callback);
ros::Subscriber takeMedication_sub = n.subscribe<project1::move>("robot_0/takeMedication",1000, takeMedication_callback);
ros::Subscriber tableToSofa_sub = n.subscribe<project1::move>("robot_0/tableToSofa",1000, tableToSofa_callback);
ros::Subscriber converseWithCaregiver_sub = n.subscribe<project1::move>("robot_0/converseWithCaregiver",1000, converseWithCaregiver_callback);
ros::Subscriber acceptEntertainment_sub = n.subscribe<project1::move>("robot_0/acceptEntertainment",1000, acceptEntertainment_callback);
ros::Subscriber acceptCompanionship_sub = n.subscribe<project1::move>("robot_0/acceptCompanionship",1000, acceptCompanionship_callback);
ros::Subscriber goToBed_sub = n.subscribe<project1::move>("robot_0/goToBed",1000, goToBed_callback);
ros::Subscriber goToAmbulance_sub = n.subscribe<project1::move>("robot_0/goToAmbulance",1000, goToAmbulance_callback);
ros::Subscriber goToSleep_sub = n.subscribe<project1::move>("robot_0/goToSleep",1000, goToSleep_callback);

//ros::Subscriber residentIllsub = n.subscribe<std_msgs::String>("robot_0/ill", 1000, illCallback);

ros::Rate loop_rate(loopRate);

//a count of how many messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel;

project1::move coord;


while (ros::ok())
{
	//messages to stage
	RobotNode_cmdvel.linear.x = linear_x;
	RobotNode_cmdvel.angular.z = angular_z;
        
	//publish the message
	RobotNode_stage_pub.publish(RobotNode_cmdvel);
	setOrientation();
	ros::spinOnce();

	coord.x = px;
	coord.y = py;
	coord.theta = theta;	
	coordPublisher.publish(coord);

	loop_rate.sleep();
	++count;

}

return 0;

}
