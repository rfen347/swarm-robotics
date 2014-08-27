#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <project1/move.h>
#include "math.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

int loopRate = 10;
double posAllowance = 0.005;

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
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_7/cmd_vel",1000); 

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

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	

	px = 6.0 + msg.pose.pose.position.x;
	py =-8.0 + msg.pose.pose.position.y;

	//ROS_INFO("Current x robot 7 is: %f", px);
	//ROS_INFO("Current y robot 7 is: %f", py);

}
void dangerCallBack(){
	ROS_INFO("TOO CLOSE!!!!");
}





void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
}

void chatterCallback(std_msgs::String Mo){
	if (Mo.data == "wake up"){

		ROS_INFO("caregiver message received");
	}
}


//Receive co-ordinates from the robot nodes and calculates the distances between them and this robot.
void coordinateCallBack(project1::move mo)
{	
	double delta_x;
	double delta_y;
	double distance;
	delta_x = px - mo.x;
	delta_y = py - mo.y;
	distance = sqrt(delta_x*delta_x + delta_y*delta_y);


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
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_7/cmd_vel",1000);


	



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
	 

	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_7/cmd_vel",1000); 


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

//Schedule to call when resident is to take a shower
void helpShower(){
	ROS_INFO("Caregiver helps resident take a shower");

	navigate(0,1);
	navigate(1,4.5);
	navigate(2,10.5);
	navigate(1,3);
	navigate(2,5);
	navigate(3,3);
	// Spin to show that caregiver is helping the resident shower.
	spin(40);


}

//Schedule to call when resident is to eat a meal
void helpEat(){
	ROS_INFO("Caregiver helps resident to eat meal");

	navigate(1,3);
	navigate(0,5);
	navigate(3,1);
	navigate(0,3);
	// Spin to show that caregiver is helping the resident eat.
	spin(40);

}

//Schedule to call when resident is to exercise
void helpExercise(){
	ROS_INFO("Caregiver helps resident with exercise");
	navigate(3,1.5);
	navigate(0,6);
	navigate(2,9);
	navigate(0,9);
	navigate(2,9);
}

//Schedule to call when resident needs conversation or moral support
void giveMoralSupport(){
	ROS_INFO("Caregiver has conversation with resident and gives moral support");

	navigate(1,2);
	navigate(0,3);
	navigate(1,4.5);
	navigate(0,1);
	// Spin to show that caregiver is talking to resident.
	spin(50);
	// Then leave the house.
	navigate(0,3.5);
	navigate(3,7);
	navigate(0,3);
	navigate(3,4.5);
	navigate(2,1);
}

void helpShower_callback(project1::move) {
	helpShower();
}

void helpEat_callback(project1::move) {
	helpEat();
}

void helpExercise_callback(project1::move) {
	helpExercise();
}

void giveMoralSupport_callback(project1::move) {
	giveMoralSupport();
}

int main(int argc, char **argv)
{

 //initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = 0;
	px = 6.0;
	py = -8.0;
	
	//Initial velocity
	linear_x = 0;
	angular_z = 0;
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "RobotNode7");


ros::init(argc, argv, "RobotNode8");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage

ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_7/cmd_vel",1000);
ros::Publisher coordPublisher= n.advertise<project1::move>("robot_7/coord",1000);   
ros::Publisher rmo= n.advertise<project1::move>("robot_7/rmove",1000);    


//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_7/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_7/base_scan",000,StageLaser_callback);

 


//ros::Subscriber 1StageOdo_sub = n.subscribe<nav_msgs::Odometry>("globalrobot/odom",1000, poop);

// subscribe from schedule node
ros::Subscriber sub_helpShower = n.subscribe<project1::move>("robot_7/helpShower",1000, helpShower_callback);	
ros::Subscriber sub_helpEat = n.subscribe<project1::move>("robot_7/helpEat",1000, helpEat_callback);	
ros::Subscriber sub_helpExercise = n.subscribe<project1::move>("robot_7/helpExercise",1000, helpExercise_callback);	
ros::Subscriber sub_giveMoralSupport = n.subscribe<project1::move>("robot_7/giveMoralSupport",1000, giveMoralSupport_callback);	

ros::Subscriber residentCoordSub = n.subscribe<project1::move>("robot_0/coord",1000, coordinateCallBack);	
ros::Subscriber cookingCoordSub = n.subscribe<project1::move>("robot_1/coord",1000, coordinateCallBack);	
ros::Subscriber vistorCoordSub = n.subscribe<project1::move>("robot_2/coord",1000, coordinateCallBack);	
ros::Subscriber scheduleCoordSub = n.subscribe<project1::move>("robot_3/coord",1000, coordinateCallBack);	
ros::Subscriber medicalCoordSub = n.subscribe<project1::move>("robot_4/coord",1000, coordinateCallBack);	
ros::Subscriber entertainmentCoordSub = n.subscribe<project1::move>("robot_5/coord",1000, coordinateCallBack);	
ros::Subscriber companionshipCoordSub = n.subscribe<project1::move>("robot_6/coord",1000, coordinateCallBack);	
ros::Subscriber relativeCoordSub = n.subscribe<project1::move>("robot_8/coord",1000, coordinateCallBack);	
ros::Subscriber doctorCoordSub = n.subscribe<project1::move>("robot_9/coord",1000, coordinateCallBack);	
ros::Subscriber nurseCoordSub = n.subscribe<project1::move>("robot_10/coord",1000, coordinateCallBack);	

ros::Rate loop_rate(loopRate);

//a count of howmany messages we have sent
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
 
	coord.x = px;
	coord.y = py;
	coord.theta = theta;	
	coordPublisher.publish(coord);


	ros::spinOnce();

	loop_rate.sleep();
	++count;
	
}

return 0;

}
