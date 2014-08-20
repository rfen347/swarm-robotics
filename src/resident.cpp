#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include "math.h"
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

void move(){
	linear_x=2;
}

void moveReverse(){
	linear_x=-2;
}

void stopMove(){
	linear_x=0;
}

void spin(){
	angular_z=2;
}

void stopSpin(){
	angular_z=0;
}

void navigate(char[] direction, float distance)
//Inputs the direction to move (North, East, South or West) and the distance to move by. The robot will carry out this movement.
{
	// Determine the current angle.

	if (char[]=={'n','o','r','t','h'}){
		// Determine the shortest rotation to make the robot face North (90 degrees). Maybe consider reverse movement too?
		// Actually carry out the rotation.
		// Determine the destination co-ordinates.

		// move();
		// while(true){
			// if(px or py has reached destination){
				// stopMove();
				// return 0;
			// }
		// }
	}else if (char[]=={'e','a','s','t'}){
		// Determine the shortest rotation to make the robot face East (0 degrees). Maybe consider reverse movement too?
		// Actually carry out the rotation.
		// Determine the destination co-ordinates.

		// move();
		// while(true){
			// if(px or py has reached destination){
				// stopMove();
				// return 0;
			// }
		// }
	}else if (char[]=={'s','o','u','t','h'}){
		// Determine the shortest rotation to make the robot face South (-90 degrees). Maybe consider reverse movement too?
		// Actually carry out the rotation.
		// Determine the destination co-ordinates.

		// move();
		// while(true){
			// if(px or py has reached destination){
				// stopMove();
				// return 0;
			// }
		// }
	}else{
		// Determine the shortest rotation to make the robot face West (180 or -180 degrees). Maybe consider reverse movement too?
		// Actually carry out the rotation.
		// Determine the destination co-ordinates.

		// move();
		// while(true){
			// if(px or py has reached destination){
				// stopMove();
				// return 0;
			// }
		// }
	}
}

void wakeUp()
{
	// Triggered by schedule.
	// Navigate to sofa, and then stop.
	// Navigate from (-6.5, 4.5) to (-3.5, 4.5), which is 3 units East.
	move();
	while(true){
		if(px>-3){
			stopMove();
			return;
		}
	}
}

void getReadyToEat()
{
	// Triggered by robot message.
	// Navigate from bedroom to dining table, and then stop.
	// Navigate from (-3.5, 4.5) to (-3.5,-1.0), which is 5.5 units South.
	// Navigate from (-3.5, -1.0) to (-0.5, -1.0), which is 3 units East.
}


void eat()
{
	// Spin to show that the resident is eating.
	spin();
}

void stopEating()
{
	// Stop spinning to show that the resident has stopped eating.
	stopSpin();
}

void StageOdom_callback(nav_msgs::Odometry msg)
{
	//This is the call back function to process odometry messages coming from Stage. 	
	px = -6.5 + msg.pose.pose.position.x;
	py = 4.5 + msg.pose.pose.position.y;
}

void StageLaser_callback(sensor_msgs::LaserScan msg)
{
	//This is the callback function to process laser scan messages
	//you can access the range data from msg.ranges[i]. i = sample number
	
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
	linear_x = 2;
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

ros::Rate loop_rate(10);

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
	
	ros::spinOnce();

	//ROS_INFO("Cycle %i - Resident co-ordinates - (%f, %f)",count,px,py);

	loop_rate.sleep();
	++count;


	// //Once the resident wakes up at a given time, make him move foward,
	// //In order to start leaving the room
	// if(count==10){
	// 	ROS_INFO("ACTIVITY - Resident wakes up");
	// 	linear_x = 2;
	// }
	// //Stop moving foward and Rotate 90 degrees clockwise
	// if(count==40){
	// 	angular_z = - M_PI / 2;
	// 	linear_x = 0;
	// } 
	// //Stop rotating and move foward out of the room
	// if(count==50){
	// 	angular_z = 0;
	// 	linear_x = 2;
	// }
	// //Once out of the room, stop moving and rotate 90 degrees anti-clockwise
	// if(count==100){
	// 	angular_z = M_PI / 2;;
	// 	linear_x = 0;
	// }
	// //Stop rotating and move foward towards the table
	// if(count==110){
	// 	angular_z = 0;
	// 	linear_x = 2;
	// }
	// //Stop moving before the resident hits the table 
	// //and rotate 90 degrees anti-clockwise
	// if(count==140){
	// 	angular_z = M_PI / 2;
	// 	linear_x = 0;
	// }

	// //Stop rotating and move foward towards the lounge
	// if(count==150){
	// 	angular_z = 0;
	// 	linear_x = 2;
	// }
	// //Stop moving foward and rotate 90 degrees clockwise
	// if(count==190){
	// 	angular_z = - M_PI / 2;
	// 	linear_x = 0;
	// }
	// //Stop rotating and move foward to go between the couch 
	// //and the tv
	// if(count==200){
	// 	angular_z = 0;
	// 	linear_x = 2;
	// }

	// //While infront of the tv, shake left and right to represent
	// //that the resident is watching tv. Log when the resident starts 
	// //and stops watching tv.
	// if(count > 210 && count < 640){
	// 	if (count == 211) {
	// 		ROS_INFO("ACTIVITY - Resident is watching TV");
	// 	}
	// 	if (count == 639) {
	// 		ROS_INFO("ACTIVITY - Resident has finished watching TV");
	// 	}
	// 	if (count % 2 < 1) {
	// 		linear_x = -10;
	// 	} else {linear_x = 10;}
	// }

	// //Move backwards away to go away from the tv
	// if(count==640){
	// 	angular_z = 0;
	// 	linear_x = -2;
	// }
	// //Rotate 90 degrees clockwise to face the direction of the dining room
	// if(count==660){
	// 	angular_z = - M_PI / 2;
	// 	linear_x = 0;
	// }
	// //Stop rotating and move towards the dining room
	// if(count==670){
	// 	angular_z = 0;
	// 	linear_x = 2;
	// }
	// //Stop moving foward when the resident is next to the table
	// //and rotate anti-clockwise to face the table
	// if(count==720){
	// 	angular_z = M_PI / 2;
	// 	linear_x = 0;
	// }
	// //Stop rotating and move foward towards the table
	// if(count==730){
	// 	angular_z = 0;
	// 	linear_x = 2;
	// }
	// //Stop moving and rotate to represent eating
	// //and log that the resident is eating
	// if(count==735){
	// 	ROS_INFO("ACTIVITY - Resident is eating");
	// 	angular_z = - M_PI / 2;
	// 	linear_x = 0;
	// }
	// //Stop rotating and move fowards (now facing away from the table)
	// //to stop eating and log that the resident finished eating
	// if(count==795){
	// 	ROS_INFO("ACTIVITY - Resident has finished eating");
	// 	angular_z = 0;
	// 	linear_x = 2;
	// }
	// //Stop moving and rotate 90 degrees clockwise to face the door of the bedroom
	// if(count==820){
	// 	angular_z = -M_PI / 2;
	// 	linear_x = 0;
	// }
	// //Stop rotating and move foward to enter the bedroom
	// if(count==830){
	// 	angular_z = 0;
	// 	linear_x = 2;
	// }
	// //Stop moving foward and rotate 90 degrees anti-clockwise to face the bed
	// if(count==890){
	// 	angular_z = M_PI / 2;
	// 	linear_x = 0;
	// }
	// //Stop rotating and move foward towards the bed
	// if(count==900){
	// 	angular_z = 0;
	// 	linear_x = 2;
	// }

	// //Stop at the bed (the original starting position)
	// //and rotate back to the original way that the resident was facing
	// if(count==932){
	// 	ROS_INFO("ACTIVITY - Resident goes to bed");
	// 	angular_z = - M_PI / 2;
	// 	linear_x = 0;
	// }
	// //Stop all resident movement while the resident sleeps
	// if(count==952){
	// 	angular_z = 0;
	// }
	
	// //Reset the day
	// if(count==1050){
	// 	count = 0;
	// }

	// ROS_INFO("Cycle %i - Resident co-ordinates - (%f,%f)",count,px,py);

	wakeUp();
}

return 0;

}
