#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sstream>
#include <project1/move.h>
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

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = 6.0 + msg.pose.pose.position.x;
	py =-8.0 + msg.pose.pose.position.y;

	ROS_INFO("Current x robot 7 is: %f", px);
	ROS_INFO("Current y robot 7 is: %f", py);

}
void dangerCallBack(){
	ROS_INFO("TOO CLOSE!!!!");
}



void collision_StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 
	double delta_x;
	double delta_y;
	//double distance;
	double rx;
	double ry;	
	rx = 6.0 + msg.pose.pose.position.x;
	ry = -7.0 + msg.pose.pose.position.y;
	delta_x = px - rx;
	delta_y = py - ry;
	//distance = sqrt(delta_x*delta_x + delta_y*delta_y);
	/*if (distance<1.5){
		dangerCallBack();
}*/
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
	//linear_x = 2;
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
}

//Schedule to call when resident is to eat a meal
void helpEat(){
	ROS_INFO("Caregiver helps resident to eat meal");
}

//Schedule to call when resident is to exercise
void helpExercise(){
	ROS_INFO("Caregiver helps resident with exercise");
}

//Schedule to call when resident needs conversation or moral support
void giveMoralSupport(){
	ROS_INFO("Caregiver has conversation with resident and gives moral support");
}

void coordinateCallback(project1::move mo)
{
	ROS_INFO("Caregiver sees robot at %f %f %f", mo.x, mo.y, mo.theta);

}

//void poop(nav_msgs::Odometry msg){
//	ROS_INFO("%f", msg.pose.pose.position.x);
//}

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

ros::Publisher rmo= n.advertise<project1::move>("robot_7/rmove",1000);    


//subscribe to listen to messages coming from stage
ros::Subscriber StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_7/odom",1000, StageOdom_callback);
ros::Subscriber StageLaser_sub = n.subscribe<sensor_msgs::LaserScan>("robot_7/base_scan",000,StageLaser_callback);

 


//ros::Subscriber 1StageOdo_sub = n.subscribe<nav_msgs::Odometry>("globalrobot/odom",1000, poop);

ros::Subscriber collision_StageOdo_sub = n.subscribe<nav_msgs::Odometry>("robot_2/odom",1000, collision_StageOdom_callback);

ros::Subscriber sub = n.subscribe<std_msgs::String>("robot_7/bbb", 1000, chatterCallback);

ros::Rate loop_rate(loopRate);

//a count of howmany messages we have sent
int count = 0;

////messages
//velocity of this RobotNode
geometry_msgs::Twist RobotNode_cmdvel; 

project1::move coord;

ros::Publisher coordinatePublisher= n.advertise<project1::move>("robot_7/coord",1000);  

ros::Subscriber residentcoordSub = n.subscribe<project1::move>("robot_0/coord",1000, coordinateCallback);
ros::Subscriber cookingcoordSub = n.subscribe<project1::move>("robot_1/coord",1000, coordinateCallback);	
ros::Subscriber friendcoordSub = n.subscribe<project1::move>("robot_2/coord",1000, coordinateCallback);	
ros::Subscriber medicalcoordSub = n.subscribe<project1::move>("robot_4/coord",1000, coordinateCallback);
ros::Subscriber entertainmentcoordSub = n.subscribe<project1::move>("robot_5/coord",1000, coordinateCallback);	
ros::Subscriber companionshipcoordSub = n.subscribe<project1::move>("robot_6/coord",1000, coordinateCallback);	
//ros::Subscriber caregivercoordSub = n.subscribe<project1::move>("robot_7/coord",1000, coordinateCallback);	
ros::Subscriber relativecoordSub = n.subscribe<project1::move>("robot_8/coord",1000, coordinateCallback);
ros::Subscriber doctorcarecoordSub = n.subscribe<project1::move>("robot_9/coord",1000, coordinateCallback);
ros::Subscriber nursecarecoordSub = n.subscribe<project1::move>("robot_10/coord",1000, coordinateCallback);		



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
	coordinatePublisher.publish(coord);

	ros::spinOnce();

	loop_rate.sleep();
	++count;

	if(count>50){
		//Mo.x = px;
		//Mo.y = py;
		//schedule_print.publish(Mo);
		//ROS_INFO("Before moving. Co-ordinates: %f,%f",px,py);

		navigate(2,3.0);
		//ROS_INFO("MOVE1. Co-ordinates: %f,%f",px,py);
		navigate(1,1.0);
		//ROS_INFO("MOVE2. Co-ordinates: %f,%f",px,py);
		navigate(0,3.0);
		//ROS_INFO("MOVE3. Co-ordinates: %f,%f",px,py);
		navigate(3,1.0);

		//ROS_INFO("After moving. Co-ordinates: %f,%f",px,py);

	}
}

return 0;

}
