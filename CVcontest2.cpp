#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <func_header.h>

#include <eStop.h>

//Tutorial Codee
#include <stdio.h>
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"

using namespace cv;
using namespace cv::xfeatures2d;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float x;
float y;
float phi;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
	phi = tf::getYaw(msg.pose.pose.orientation);
    	x = msg.pose.pose.position.x;
    	y = msg.pose.pose.position.y;
}

void readme(){
	std::cout<<" Usage: ./SURF_descriptor <img1> <img2>" << std::endl;
}

bool isMatch(Mat img_object, Mat img_scene){
    float avgDist=0;
		int count=0;
		float sumDist=0;
		int minHessian = 400;
		Ptr<SURF> detector = SURF::create(minHessian);
		vector<KeyPoint> keypoints_object, keypoints_scene;
		Mat descriptors_object, descriptors_scene;
		detector->detectAndCompute(img_object, Mat(), keypoints_object, descriptors_object);
		detector->detectAndCompute(img_scene, Mat(), keypoints_scene, descriptors_scene);
		FlannBasedMatcher matcher;
		std::vector< DMatch > matches;
		matcher.match( descriptors_object, descriptors_scene, matches );
		double max_dist = 0; double min_dist = 100;
		for(int i=0; i<descriptors_object.rows; i++){
			double dist = matches[i].distance;
			if(dist<min_dist) min_dist = dist;
			if(dist>max_dist) max_dist = dist; 
		}
		printf("-- Max dist : %f \n", max_dist );
		printf("-- Min dist : %f \n", min_dist );
		std::vector< DMatch > good_matches;
		for(int i=0; i< descriptors_object.rows; i++)
			if(matches[i].distance < 3*min_dist)
				good_matches.push_back(matches[i]);
		Mat img_matches;
		drawMatches( img_object, keypoints_object, img_scene, keypoints_scene, good_matches, img_matches, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		std::vector<Point2f> obj;
		std::vector<Point2f> scene;
		for(int i=0; i< good_matches.size(); i++){
			obj.push_back(keypoints_object[ good_matches[i].queryIdx].pt);
			scene.push_back(keypoints_scene[ good_matches[i].trainIdx].pt);
			count++;
			sumDist+=good_matches[i].distance;
		}
		avgDist=sumDist/count;
		Mat H = findHomography(obj, scene, RANSAC);
		std::vector<Point2f> obj_corners(4);
		obj_corners[0] = cvPoint(0,0);
		obj_corners[1] = cvPoint(img_object.cols, 0);
		obj_corners[2] = cvPoint(img_object.cols, img_object.rows);	
		obj_corners[3] = cvPoint(0, img_object.rows);
		std::vector<Point2f> scene_corners(4);	
		perspectiveTransform(obj_corners, scene_corners, H);
		line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0,255,0),4);
		line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0,255,0),4);
		line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0,255,0),4);
		line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0,255,0),4);
		imshow("Good Matches & Object Detection", img_matches);
		waitKey(0);
		
		cout << "There are " << count << " matches, with an average distance of " << avgDist << "."<< endl;
		if(count<30&&avgDist<0.2)
			return true;
		return false;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
  	teleController eStop;

	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, poseCallback);
	
	vector<vector<float> > coord;
	vector<cv::Mat> imgs_track;	
	if(!init(coord, imgs_track)) return 0;

	for(int i = 0; i < coord.size(); ++i){
		cout << i << " x: " << coord[i][0] << " y: " << coord[i][1] << " z: " << coord[i][2] << endl;
	}

	imageTransporter imgTransport("/camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); // For Kinect
	
	//Stores which image is at which location: -1 = no data, 0 = blank, 1 = raisin, 2 = cinnamon, 3 = rice
	int tag [5] = {-1,-1,-1,-1,-1};
	int currLoc = 0;

	while(ros::ok()){
		ros::spinOnce();
  		//.....**E-STOP DO NOT TOUCH**.......
   		eStop.block();
    		//....................................

    		//Tutorial Code

		Mat img_raisin = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/pics/tag1.jpg", IMREAD_GRAYSCALE );
		Mat img_cinnamon = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/pics/tag2.jpg", IMREAD_GRAYSCALE );
		Mat img_rice = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/pics/tag3.jpg", IMREAD_GRAYSCALE );

		Mat img_cam = imgTransport.getImg();
		if(!img_raisin.data||!img_cinnamon.data||!img_rice.data||!img_cam.data )
			{std::cout<< " --(!) Error reading images " << std::endl; }
		else{
			if(isMatch(img_raisin,img_cam)){
			  tag[currLoc]=1;  
				cout << "This is a raisin." << endl;
			} 
			else if(isMatch(img_cinnamon,img_cam)){
			  tag[currLoc]=2;
				cout << "This is a cinnamon." << endl;
			}
			else if(isMatch(img_rice,img_cam)){
			  tag[currLoc]=3;
				cout << "This is a rice." << endl;
			} 
			else{
			  tag[currLoc]=0;
			  cout << "This is a blank." << endl;
			}
		}
	}
	cout << "The tags are as follows:" << endl;
	for(int i=1;i<=5;i++){
		cout << "Tag " << i << " is " ;
		switch(tag[i]){
			case 0 : cout << "blank." << endl;
			case 1 : cout << "Raisin Bran." << endl;
			case 2 : cout << "Cinnamon Toast Crunch." << endl;
			case 3 : cout << "Rice Krispies." << endl;
		}
	}
	
	return 0;
}

//-------------------------move robot function---------------
bool moveToGoal(float xGoal, float yGoal, float phiGoal){

	//define a client for to send goal requests to the move_base server through a SimpleActionClient
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;

	//set up the frame parameters
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	/* moving towards the goal*/
    	geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(phiGoal);

	goal.target_pose.pose.position.x =  xGoal;
	goal.target_pose.pose.position.y =  yGoal;
	goal.target_pose.pose.position.z =  0.0;
	goal.target_pose.pose.orientation.x = 0.0;
	goal.target_pose.pose.orientation.y = 0.0;
	goal.target_pose.pose.orientation.z = phi.z;
	goal.target_pose.pose.orientation.w = phi.w;

	ROS_INFO("Sending goal location ...");
	ac.sendGoal(goal);

	ac.waitForResult();

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		ROS_INFO("You have reached the destination");
		return true;
	}
	else{
		ROS_INFO("The robot failed to reach the destination");
		return false;
	}

}


