#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <eStop.h>

#include <stdio.h>
#include <cmath>

#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/OccupancyGrid.h>

using namespace std;

double angular, linear;

//for odom
double posX, posY, yaw, yawCurr=0;
int yawRead=0;
double pi = 3.1416;

double bumperLeft = 0, bumperCenter = 0, bumperRight = 0;

//for laser
double laserRange = 10;
int laserSize = 0, laserOffset = 0, desiredAngle = 5;

//for occupancy
int mapWidth=0, mapHeight=0;
double mapResolution=0.05, mapX=0, mapY=0;
int randomMap = 0;

unsigned int indexPos=0;
unsigned int point1=0;
int p1data=0;

unsigned int blackNE=0, blackSE=0, blackSW=0, blackNW=0, unknownNE=0, unknownSE=0, unknownSW=0, unknownNW=0;

const unsigned int detectionRadius=20;

//flag for turning
int turnflag = 0;

void bumperCallback(const kobuki_msgs::BumperEvent msg){
	if(msg.bumper==0)
		bumperLeft = !bumperLeft;
	else if(msg.bumper == 1)
		bumperCenter = !bumperCenter;
	else if(msg.bumper == 2)
		bumperRight = !bumperRight;
	ROS_INFO("BUMPER DATA COLLECTED");
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
	laserSize = (msg->angle_max - msg->angle_min)/msg->angle_increment;
	laserOffset = desiredAngle*pi/(180*msg->angle_increment);
	laserRange = 11;

	if(desiredAngle*pi/180 < msg->angle_max && -desiredAngle*pi/180 > msg->angle_min){
		for(int i = laserSize/2 - laserOffset; i < laserSize/2 + laserOffset; i++){
			if(laserRange > msg->ranges[i])
				laserRange = msg->ranges[i];
		}
	}
	else{
		for(int i = 0; i < laserSize; i++){
			if(laserRange > msg->ranges[i])
				laserRange = msg->ranges[i];
		}
	}
	
	if(laserRange == 11)
		laserRange = 0;
	//ROS_INFO("LASER DATA COLLECTED");

	//ROS_INFO("Size of laser scan array: %i and size of offset: %i", laserSize, laserOffset);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
	posX = msg->pose.pose.position.x;
	posY = msg->pose.pose.position.y;
	yaw = tf::getYaw(msg->pose.pose.orientation);
	//ROS_INFO("ODOM DATA COLLECTED");

	//ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw*180/pi);
}


void occupancyCallback(const nav_msgs::OccupancyGrid& msg){
	mapWidth = msg.info.width;
	mapHeight = msg.info.height;
	mapResolution = msg.info.resolution;
	mapX = msg.info.origin.position.x;
	mapY = msg.info.origin.position.y;
	randomMap = msg.data[msg.info.width*msg.info.height-1];
	
	/*//Find number of unknown points (for debugging)
	p1data=0;
	for(int i=0; i<msg.info.width*msg.info.height; i++){
		if(msg.data[i]==-1){
			point1 = i;
			p1data++;			
			//break;
		}
	}*/
	
	//Check that there is a map to check
	/*if(mapWidth>detectionRadius&&mapHeight>detectionRadius&&indexPos>detectionRadius*detectionRadius){
		blackNE=0, blackSE=0, blackSW=0, blackNW=0, unknownNE=0, unknownSE=0, unknownSW=0, unknownNW=0;
		//NE quadrant from robot
		for(int i=0; i<detectionRadius; i++){
			for(int j=0; j<detectionRadius; j++){
				if(indexPos+j+mapWidth*i>0){
					if(msg.data[indexPos+j+mapWidth*i]>0)
						blackNE++;
					if(msg.data[indexPos+j+mapWidth*i]==-1)
						unknownNE++;
				}
			}
		}

		//SE quadrant from robot
		for(int i=0; i<detectionRadius; i++){
			for(int j=0; j<detectionRadius; j++){
				if(indexPos+j-mapWidth*i>0){
					if(msg.data[indexPos+j-mapWidth*i]>0)
						blackSE++;
					if(msg.data[indexPos+j-mapWidth*i]==-1)
						unknownSE++;
				}
			}
		}

		//SW quadrant from robot
		for(int i=0; i<detectionRadius; i++){
			for(int j=0; j<detectionRadius; j++){
				if(indexPos-j-mapWidth*i>0){
					if(msg.data[indexPos-j-mapWidth*i]>0)
						blackSW++;
					if(msg.data[indexPos-j-mapWidth*i]==-1)
						unknownSW++;
				}
			}
		}

		//NW quadrant from robot
		for(int i=0; i<detectionRadius; i++){
			for(int j=0; j<detectionRadius; j++){
				if(indexPos-j+mapWidth*i>0){
					if(msg.data[indexPos-j+mapWidth*i]>0)
						blackNW++;
					if(msg.data[indexPos-j+mapWidth*i]==-1)
						unknownNW++;
				}
			}
		}
	}*/
	
		
	//ROS_INFO("OCCUPANCY DATA COLLECTED");
	//ROS_INFO("Width: %i, Height: %i, Resolution: %f, Origin: (%f,%f), Random Map: %d", msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y, msg.data[msg.info.width*msg.info.height-1]);
}
void turn(double ang, ros::Publisher velocityPub, geometry_msgs::Twist velocity){
			yawCurr=yaw;
			linear = 0;
			angular = -pi/6; 
			velocity.angular.z = angular;
  			velocity.linear.x = linear;

			//Turn while loop
			while (abs(yaw-yawCurr)<ang*pi/180){
				ros::spinOnce();
  				velocityPub.publish(velocity);
				if(yawRead!=yaw){
					//ROS_INFO("Only %f left. %f", 90-abs(yaw-yawCurr)*180/pi, laserRange);
				}
				else {
					yawRead=(int) yaw;
				}
			}

				ROS_INFO("SUCCESSFUL TURN!!!!");
				angular = 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	teleController eStop;

	ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
	ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
	ros::Subscriber odom = nh.subscribe("odom",1,odomCallback); //No &?
	ros::Subscriber occ = nh.subscribe("map",10,&occupancyCallback);
	

	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);

	angular = 0.0;
	linear = 0.0;
	geometry_msgs::Twist vel;

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//fill with your code
		ROS_INFO("Position: (%f,%f) Orientation: %f degrees Range: %f", posX, posY, yaw*180/pi, laserRange);
		indexPos = (posY-mapY)/mapResolution*mapWidth + (posX-mapX)/mapResolution;
		//ROS_INFO("Robot Index: %d", indexPos);	
		//ROS_INFO("Blacks: %d, %d, %d, %d. Unknowns: %d, %d, %d, %d.", blackNE, blackSE, blackSW, blackNW, unknownNE, unknownSE, unknownSW, unknownNW);
		//ROS_INFO("Non clear: %d, Last @: %d", p1data, point1);	
		
		if(laserRange < 0.5){
			turnflag = 1;
			ROS_INFO("I NEED TO TURN!!!! %f",laserRange);
		}
		else if(laserRange > 2){
			turnflag = 0;
		}
		

		if(turnflag==0){
			angular = 0.0;
			linear = 0.2;
		}
		/*else if(laserRange==0){
			angular = 0.0;
			linear = 0.0;
			ROS_INFO("NO LASER DATA!!!");
		}*/
		else{
			turn(22.5, vel_pub, vel);
				//turnflag=0;
		}

		if(bumperRight)
		{
			int pos1X = posX;
			int pos1Y = posY;
			double dist = sqrt((posX-pos1X)*(posX-pos1X)+(posY-pos1Y)*(posY-pos1Y));
			while(dist < 2){
				angular = 0.0;
				linear = -0.2;
				vel.angular.z = angular;
  				vel.linear.x = linear;
				ros::spinOnce();
  				vel_pub.publish(vel);
			}
			angular = 0.0;
			linear = 0.0;
			vel.angular.z = angular;
  			vel.linear.x = linear;
			ros::spinOnce();
  			vel_pub.publish(vel);
			turn(10, vel_pub, vel);
		}
		if(bumperLeft)
		{
			int pos1X = posX;
			int pos1Y = posY;
			double dist = sqrt((posX-pos1X)*(posX-pos1X)+(posY-pos1Y)*(posY-pos1Y));
			while(dist < 2){
				angular = 0.0;
				linear = -0.2;
				vel.angular.z = angular;
  				vel.linear.x = linear;
				ros::spinOnce();
  				vel_pub.publish(vel);
			}
			angular = 0.0;
			linear = 0.0;
			vel.angular.z = angular;
  			vel.linear.x = linear;
			ros::spinOnce();
  			vel_pub.publish(vel);
			turn(-10, vel_pub, vel);
		}
		if(bumperCenter)
		{
			int pos1X = posX;
			int pos1Y = posY;
			double dist = sqrt((posX-pos1X)*(posX-pos1X)+(posY-pos1Y)*(posY-pos1Y));
			while(dist < 2){
				angular = 0.0;
				linear = -0.2;
				vel.angular.z = angular;
  				vel.linear.x = linear;
				ros::spinOnce();
  				vel_pub.publish(vel);
			}
			angular = 0.0;
			linear = 0.0;
			vel.angular.z = angular;
  			vel.linear.x = linear;
			ros::spinOnce();
  			vel_pub.publish(vel);
			turn(90, vel_pub, vel);
		}		


		/*if(posX < 0.5 && yaw < pi/12 && !bumperRight && !bumperCenter && !bumperLeft && laserRange > 0.7)
		{
			angular = 0.0;
			linear = 0.2;
		}
		else if(posX > 0.4 && yaw < pi/2 && !bumperRight && !bumperCenter && !bumperLeft && laserRange < 0.5)
		{
			angular = pi/6;
			linear = 0.0;
		} 
		else if(laserRange > 1.0 && !bumperRight && !bumperCenter && !bumperLeft)
		{
			if(yaw < 17*pi/36 || posX > 0.6)
			{
				angular = pi/12;
				linear = 0.1;
			}
			else if(yaw > 19*pi/36 || posX < 0.4)
			{
				angular = -pi/12;
				linear = 0.1;
			}
			else
			{
				angular = 0;
				linear = 0.1;
			}
		}
		else 
		{
			angular = 0.0;
			linear = 0.0;
		}*/

  		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);
	}

	return 0;
}
