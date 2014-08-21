#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>

#include <sstream>
#include "math.h"

//velocity of the robot
double linear_x;
double angular_z;

//pose of the robot
double px;
double py;
double theta;

void StageLaser_callback(sensor_msgs::LaserScan msg);
void StageOdom_callback(nav_msgs::Odometry msg);

bool isClear (){

}

// bool isClear(    ?   all the robots   ?      )

// isClear
// method 1 = check the path is clear for every robot
// get location and dir of robot -- GRID
// return boolean

// if true keep moving otherwise -> go to method 2

// method 2 = checks which robot is bigger. Bigger robot gets first priority and moves before the small robot

void detectCollision (int Robot1, int Robot2){

// Check robot number 
if (Robot1 > Robot2)
{
	//NAVIGATION

} else

{
// Robot1 moves 

}
	//NAVIGATION

// Robot2 moves 

}

