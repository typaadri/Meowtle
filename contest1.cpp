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
double posX, posY, yaw, yawCurr=0, straightYaw;
int yawRead=0;
double pi = 3.1416;

double bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
double pos1X = 0;
double pos1Y = 0;
double dist=0;
bool goRight=0;

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

// Bumperflags
bool bRight=0, bLeft=0, bCenter=0;

//flag for turning
int turnflag = 0;

void bumperCallback(const kobuki_msgs::BumperEvent msg){
	if(msg.bumper==0)
		bumperLeft = msg.state;
	else if(msg.bumper == 1)
		bumperCenter = msg.state;
	else if(msg.bumper == 2)
		bumperRight = msg.state;
	
	// initialize bumper flag
	if(bumperRight==1)
		bRight = 1;
	if(bumperLeft==1)
		bLeft = 1;
	if(bumperCenter==1)
		bCenter = 1;

	ROS_INFO("BUMPER DATA COLLECTED: %d, %d", msg.bumper, msg.state);
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
	
		
	//ROS_INFO("OCCUPANCY DATA COLLECTED");
	//ROS_INFO("Width: %i, Height: %i, Resolution: %f, Origin: (%f,%f), Random Map: %d", msg.info.width, msg.info.height, msg.info.resolution, msg.info.origin.position.x, msg.info.origin.position.y, msg.data[msg.info.width*msg.info.height-1]);
}
void turn(double ang, ros::Publisher velocityPub, geometry_msgs::Twist velocity){
			yawCurr=yaw;
			linear = 0;
			angular = -pi/8; 

			if(ang<0){
				angular=-1*angular;
				ang=-1*ang;
			}
			velocity.angular.z = angular;
  			velocity.linear.x = linear;
				

			//Turn while loop
			while (abs(yaw-yawCurr)<ang*pi/180){
				ros::spinOnce();
  				velocityPub.publish(velocity);
			}

			ROS_INFO("SUCCESSFUL TURN!!!!");
			angular = 0;
			straightYaw = yaw;
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
	
	turn(90,vel_pub,vel);
	turn(90,vel_pub,vel);
	turn(90,vel_pub,vel);
	turn(90,vel_pub,vel);
	
	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................

		//fill with your code
		
		if(bumperRight==1||bumperLeft==1||bumperCenter==1)
		{
			ROS_INFO("Bumper flag works");
			
			pos1X = posX;
			pos1Y = posY;
			dist=0;
			while(dist < 0.1){
				ROS_INFO("Backing up: %f, %f, %f", dist, pos1X, posX);
				angular = 0.0;
				linear = -0.2;
				vel.angular.z = angular;
  				vel.linear.x = linear;
  				vel_pub.publish(vel);
 				dist = sqrt((posX-pos1X)*(posX-pos1X)+(posY-pos1Y)*(posY-pos1Y));
				ROS_INFO("Backing up: %f", dist);
				ros::spinOnce();
			}
			angular = 0.0;
			linear = 0.0;
			vel.angular.z = angular;
  			vel.linear.x = linear;
  			vel_pub.publish(vel);
			//turn(10, vel_pub, vel);
			if(bRight==1)
			{
				turn(-25, vel_pub, vel);
				bRight = 0;
				ROS_INFO("Bumper R");
	
			}
			if(bLeft==1)
			{
				turn(25, vel_pub, vel);
				bLeft = 0;
				ROS_INFO("Bumper L");
			}
			if(bCenter==1)
			{
				turn(90, vel_pub, vel);
				bCenter = 0;
				ROS_INFO("Bumper C");
			}
		}
		
		else if(turnflag==0){
			//ROS_INFO("Straightness: %f", yaw-straightYaw);
			angular = 0.0;
			linear = 0.2;
			if(abs(yaw-straightYaw)>pi/180){
				if(yaw-straightYaw>0){
					angular = -pi/8;
				}
				else{
					angular = pi/8;
				}
			}
			
		}
		else{
			if(goRight){
				turn(10, vel_pub, vel);
			}
			else{
				turn(-10,vel_pub,vel);
			}
		}

		if(laserRange < 0.75 && turnflag==0){
			turnflag = 1;
			ROS_INFO("I NEED TO TURN!!!! %f",laserRange);
			goRight = rand()%2;
		}
		else if(laserRange > 1 && turnflag==1){
			turnflag = 0;

			
		}
		vel.angular.z = angular;
  		vel.linear.x = linear;

  		vel_pub.publish(vel);
	}

	return 0;
}

