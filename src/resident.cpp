#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include "math.h"
#include <string>

#include <rosgraph_msgs/Clock.h>



//#include "cookingrobot.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

void StageLaser_callback(sensor_msgs::LaserScan msg);
void StageOdom_callback(nav_msgs::Odometry msg);

int loopRate = 10; // Setting the cycle rate per second.
double posAllowance = 0.005;
// This function updates the theta field so that the robot knows which angle it is facing.
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
}

//Schedule to call for the resident to use the toilet
void useToilet(){
	ROS_INFO("Resident goes to the toilet");
}

//Schedule to call for the resident to use the sink
void useSink(){
	ROS_INFO("Resident goes to the sink");
}

//Schedule to call for the resident to shower
void shower(){
	ROS_INFO("Resident goes to the bath");
}

//Schedule to call for the resident to go to the lounge
void bathroomToLounge(){
	ROS_INFO("Resident leaves the bathroom and enters the lounge");
}

//Schedule to call for the resident to exercise
void exercise(){
	ROS_INFO("Resident exercises");	
}

//Schedule to call for the resident to go to the table
void getReadyToEat(){
	ROS_INFO("Resident goes to the table to eat");
}

// Schedule to call for the resident to eat
void eat(){
	ROS_INFO("Resident eats");
}

// Schedule to call for the resident to take medication
void takeMedication(){
	ROS_INFO("Resident takes medication");
}

// Schedule to call for the resident to go to the sofa
void tableToSofa(){
	ROS_INFO("Resident moves to the sofa");
}

// Schedule to call for the resident to talk to the caregiver
void converseWithCaregiver(){
	ROS_INFO("Resident converses with the caregiver");
}

// Schedule to call for the resident to talk to the visitors
void converseWithVisitors(){
	ROS_INFO("Resident converses with visitors");
}

// Schedule to call for the resident to accept entertainment from the entertainment robot
void acceptEntertainment(){
	ROS_INFO("Resident accepts Entertainment");
}

// Schedule to call for the resident to accept companionship robot
void acceptCompanionship(){
	ROS_INFO("Resident video chats/talks to companionship robot");
}

// Schedule to call for the resident to go to bed
void goToBed(){
	ROS_INFO("Resident goes to bed");
}

// Schedule to call for the resident to sleep
void sleep(){
	ROS_INFO("Resident is sleeping");
}

// Schedule to call for the resident to get sick
void getSick(){
	ROS_INFO("Resident is sick");
}

// Schedule to call for the resident to go to the ambulance and leave the house
void goToAmbulance(){
	ROS_INFO("Resident leaves: Too sick");
}

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	//ROS_INFO("Current x position is: %f", px);
	//ROS_INFO("Current y position is: %f", py);
	px = -6.5 + msg.pose.pose.position.x;
	py = 4.5 + msg.pose.pose.position.y;	
}

void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

//void clockCallback(rosgraph_msgs::Clock clk){
//	ROS_INFO("clock start!!!!!!");
	//linear_x = 2;
//}

void chatterCallback(std_msgs::String Mo){
	if (Mo.data == "wake up"){

		ROS_INFO("wake up message received");
	}
	//linear_x = 2;
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

//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_0/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_0/base_scan",1000,StageLaser_callback);

ros::Subscriber sub = n.subscribe<std_msgs::String>("robot_0/bbb", 1000, chatterCallback);
//ros::Subscriber clk = n.subscribe<rosgraph_msgs::Clock>("/clock", 1000, clockCallback);


ros::Rate loop_rate(loopRate);

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
	
	setOrientation();

	ros::spinOnce();

	loop_rate.sleep();
	++count;


}

return 0;

}
