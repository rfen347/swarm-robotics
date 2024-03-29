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
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_10/cmd_vel",1000); 

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
	px = 6 + msg.pose.pose.position.x;
	py = -11 + msg.pose.pose.position.y;

	//ROS_INFO("Current x position is: %f", px);
	//ROS_INFO("Current y position is: %f", py);

}


void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
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
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_10/cmd_vel",1000);
	
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
	ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_10/cmd_vel",1000); 

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

//Schedule to call when the resident gets ill
void visit(){
	ROS_INFO("Nurse visits when resident is ill");
	navigate(0,1);
	navigate(1,7.5);
	navigate(2,4);
	ROS_INFO("Nurse is getting medicine");
	// Spin to show that nurse is getting medicine.
	spin(40);
	navigate(2,6.5);
	navigate(1,6.4);
	navigate(2,3);
	// Spin to show that nurse is treating patient.
	spin(60);
	// Leave.
	ROS_INFO("Nurse leaves the house");
	navigate(0,3);
	navigate(3,6.4);
	navigate(0,10.5);
	navigate(3,7.5);
	navigate(2,1);
	//Set back to original orientation
	rotateToAngle(0);

}
//Receive co-ordinates from the robot nodes and calculates the distances between them and this robot.
void coordinateCallback(project1::move mo)
{
	double delta_x;
	double delta_y;
	double distance;
	delta_x = px - mo.x;
	delta_y = py - mo.y;
	distance = sqrt(delta_x*delta_x + delta_y*delta_y);


}

void visit_callback(project1::move){
	visit();
}

int main(int argc, char **argv)
{

 //initialize robot parameters
	//Initial pose. This is same as the pose that you used in the world file to set	the robot pose.
	theta = 0;
	px = 6;
	py = -11;
	
	//Initial velocity
	linear_x = 0;
	angular_z = 0;
	
//You must call ros::init() first of all. ros::init() function needs to see argc and argv. The third argument is the name of the node
ros::init(argc, argv, "RobotNode10");

//NodeHandle is the main access point to communicate with ros.
ros::NodeHandle n;

//advertise() function will tell ROS that you want to publish on a given topic_
//to stage
ros::Publisher RobotNode_stage_pub = n.advertise<geometry_msgs::Twist>("robot_10/cmd_vel",1000);

ros::Publisher coordinatePublisher= n.advertise<project1::move>("robot_10/coord",1000); 
 

//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_10/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_10/base_scan",1000,StageLaser_callback);


ros::Subscriber visit_sub = n.subscribe<project1::move>("robot_10/visit",1000, visit_callback);

ros::Subscriber residentcoordSub = n.subscribe<project1::move>("robot_0/coord",1000, coordinateCallback);
ros::Subscriber cookingcoordSub = n.subscribe<project1::move>("robot_1/coord",1000, coordinateCallback);	
ros::Subscriber friendcoordSub = n.subscribe<project1::move>("robot_2/coord",1000, coordinateCallback);	
ros::Subscriber medicalcoordSub = n.subscribe<project1::move>("robot_4/coord",1000, coordinateCallback);
ros::Subscriber entertainmentcoordSub = n.subscribe<project1::move>("robot_5/coord",1000, coordinateCallback);	
ros::Subscriber companionshipcoordSub = n.subscribe<project1::move>("robot_6/coord",1000, coordinateCallback);	
ros::Subscriber caregivercoordSub = n.subscribe<project1::move>("robot_7/coord",1000, coordinateCallback);	
ros::Subscriber relativecoordSub = n.subscribe<project1::move>("robot_8/coord",1000, coordinateCallback);
ros::Subscriber doctorcarecoordSub = n.subscribe<project1::move>("robot_9/coord",1000, coordinateCallback);	


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

	coord.x = px;
	coord.y = py;
	coord.theta = theta;	
	coordinatePublisher.publish(coord);

	setOrientation();

	coord.x = px;
	coord.y = py;
	coord.theta = theta;	
	coordinatePublisher.publish(coord);

	ros::spinOnce();

	loop_rate.sleep();
	++count;
}

return 0;

}
