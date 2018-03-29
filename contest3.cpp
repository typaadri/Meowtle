#include <header.h>
#include <imageTransporter.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <kobuki_msgs/BumperEvent.h>
//#include <func_header.h>

#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

#define PI 3.14159265

using namespace cv;
using namespace cv::xfeatures2d;

using namespace std;

geometry_msgs::Twist follow_cmd;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int world_state, sadpic;
//for bumper
double bumperLeft = 0, bumperCenter = 0, bumperRight = 0;
bool bRight=0, bLeft=0, bCenter=0;

void followerCB(const geometry_msgs::Twist msg){
   	follow_cmd = msg;
}

void bumperCB(const kobuki_msgs::BumperEvent msg){
	//Current Bumper States
	if(msg.bumper==0)
		bumperLeft = msg.state;
	else if(msg.bumper == 1)
		bumperCenter = msg.state;
	else if(msg.bumper == 2)
		bumperRight = msg.state;

	//Resettable bumper hit flag
	if(bumperRight==1)
		bRight = 1;
	if(bumperLeft==1)
		bLeft = 1;
	if(bumperCenter==1)
		bCenter = 1;

	ROS_INFO("BUMPER DATA COLLECTED: %d, %d", msg.bumper, msg.state);
}

//-------------------------------------------------------------

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	string path_to_sounds = "/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/";
	teleController eStop;

	//get animation images
	/*
	string packpath = ros::package::getPath("mie443_contest3");
	string filepath = packpath + "/animation/";
	cv::imread(imagepath, CV_LOAD_IMAGE_GRAYSCALE)
	*/
	//publishers
	ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop",1);
	ros::Publisher animation_pub = nh.advertise<sensor_msgs::Image>("animation",1);

	//subscribers
	ros::Subscriber follower = nh.subscribe("follower_velocity_smoother/smooth_cmd_vel", 10, &followerCB);
	ros::Subscriber bumper = nh.subscribe("mobile_base/events/bumper", 10, &bumperCB);

	imageTransporter rgbTransport("camera/image/", sensor_msgs::image_encodings::BGR8); //--for Webcam
	//imageTransporter rgbTransport("camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); //--for turtlebot Camera
	imageTransporter depthTransport("camera/depth_registered/image_raw", sensor_msgs::image_encodings::TYPE_32FC1);

	int state = 0;

	double angular = 0.0;
	double linear = 0.0;

	geometry_msgs::Twist vel;
	vel.angular.z = angular;
	vel.linear.x = linear;
  	
	Mat cat_neutral = cv::imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pics/cat_neutral.jpg", IMREAD_UNCHANGED );
	Mat cat_fear = cv::imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pics/cat_fear.jpg", IMREAD_UNCHANGED );
	Mat cat_sad = cv::imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pics/cat_sad.jpg", IMREAD_UNCHANGED );
	Mat cat_surprise = cv::imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pics/cat_surprise.jpg", IMREAD_UNCHANGED );
	Mat cat_disgust = cv::imread("/home/turtlebot/catkin_ws/src/mie443_contest3/pics/cat_disgust.jpg", IMREAD_UNCHANGED );
	if(! cat_neutral.data){
		ROS_INFO("no data");
		return -1;
	}	

	while(ros::ok()){
		ros::spinOnce();
		//.....**E-STOP DO NOT TOUCH**.......
		eStop.block();
		//...................................
		
		//ROS_INFO("%f", bumperCenter);

		ROS_INFO("%f",follow_cmd.linear.x);
		if(follow_cmd.linear.x == 0.14256){
			world_state = 1;
		}
		else if(bCenter == 1){
			world_state = 2;
			bCenter = 0;
		}

		ROS_INFO("World State: %d", world_state);

		if(world_state == 0){
			sadpic = 0;
			//fill with your code
			vel_pub.publish(follow_cmd);
			sleep(0.5);/*

			sc.playWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/cat_growl.wav");
			sleep(1.0);
			sc.stopWave("/home/turtlebot/catkin_ws/src/mie443_contest3/sounds/cat_growl.wav");
*/
			//cv::namedWindow( "Emotion", WINDOW_AUTOSIZE);
			ROS_INFO("neutral cat");
			cv::imshow( "Emotion", cat_neutral);
			waitKey(100);			

		}else if(world_state == 1){
			// lost function
			// robot rotates 90 degrees left, then 180 degrees right, then gets sad
			if (sadpic == 0){
				for(int i=0; i < 250000; i++){
					follow_cmd.linear.x = 0;
					follow_cmd.angular.z = PI/6;
					vel_pub.publish(follow_cmd);
				}
				for(int i=0; i < 500000; i++){
					follow_cmd.linear.x = 0;
					follow_cmd.angular.z = -PI/6;
					vel_pub.publish(follow_cmd);
				}
				for(int i=0; i < 250000; i++){
					follow_cmd.linear.x = 0;
					follow_cmd.angular.z = PI/6;
					vel_pub.publish(follow_cmd);
				}
				sadpic = 1;
			}else if (sadpic == 1){
				cv::imshow( "Emotion", cat_sad);
				waitKey(100);	
				world_state = 0;
			}
		}else if(world_state == 2){
			// disgust function (bumper activation)
			// back up, and show disgusted face and sound
			cv::imshow( "Emotion", cat_disgust);
			waitKey(100);	
			sleep(3);

			world_state = 0;
		}
	}

	return 0;
}
