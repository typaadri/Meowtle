#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <func_header.h>

#include <eStop.h>
#include <math.h>

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

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float x;
float y;
float phi;
float xStart, yStart, phiStart;

//Stores which image is at which location: -1 = no data, 0 = blank, 1 = raisin, 2 = cinnamon, 3 = rice
int tag [5] = {-1,-1,-1,-1,-1};
int tagAgain [5] = {-1,-1,-1,-1,-1};
int currLoc = 0;
int success[5] = {0,0,0,0,0};

//variables for finding distances between pointss
int nB = 5;
float distances[6][5] = {0};
float rCoord[3];
int order[5];
bool used[5] = {0};
float lowest;
float newCoords [5][3];
float botDistance = 0.6;


void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
	phi = tf::getYaw(msg.pose.pose.orientation);
    	x = msg.pose.pose.position.x;
    	y = msg.pose.pose.position.y;
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
		}
		int area=-1;
		if(good_matches.size()>3){
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
			//waitKey(0);
			for(int i=0; i< good_matches.size(); i++){
				count++;
				sumDist+=good_matches[i].distance;
			}
			avgDist=sumDist/count;
			cout << "There are " << count << " matches, with an average distance of " << avgDist << "."<< endl;
			int x1 = scene_corners[0].x + Point2f(img_object.cols, 0).x;
			int y1 = scene_corners[0].y + Point2f(img_object.cols, 0).y;
			int x2 = scene_corners[1].x + Point2f(img_object.cols, 0).x;
			int y2 = scene_corners[1].y + Point2f(img_object.cols, 0).y;
			int x3 = scene_corners[2].x + Point2f(img_object.cols, 0).x;
			int y3 = scene_corners[2].y + Point2f(img_object.cols, 0).y;
			int x4 = scene_corners[3].x + Point2f(img_object.cols, 0).x;
			int y4 = scene_corners[3].y + Point2f(img_object.cols, 0).y;
			area = ((x1*y2-y1*x2)+(x2*y3-y2*x3)+(x3*y4-y3*x4)+(x4*y1-y4*x1))/2;
			cout << "The area is: " << area << endl;		
		}
		else return false;

		if(area>7000&&avgDist<0.3)
			return true;
		return false;
}

// This function takes the locations of the objects and moves the robot to a set location away from them in the opposite orientation
void pathCalc(std::vector<std::vector<float> > coord){

	for (int i = 0; i < coord.size(); ++i){
		newCoords[i][0] = coord[i][0]+botDistance*cos(coord[i][2]);
		newCoords[i][1] = coord[i][1]+botDistance*sin(coord[i][2]);
		if(coord[i][2] > 0){
			newCoords[i][2] = coord[i][2] - PI;
		}else{
			newCoords[i][2] = coord[i][2] + PI;
		}
	}

	//print oldcoord table
	cout << "old co-ordinate table" << endl;
	for(int i = 0; i < coord.size(); ++i){
		cout << i << " x: " << coord[i][0] << " y: " << coord[i][1] << " z: " << coord[i][2] << endl;
	}

	//print newcoord table
	cout << endl << "new co-ordinate table" << endl;
	for(int i = 0; i < coord.size(); ++i){
		cout << i << " x: " << newCoords[i][0] << " y: " << newCoords[i][1] << " z: " << newCoords[i][2] << endl;
	}

}

// this function calculates the distances between all known object locations
void distCalcs(float newCoords[][3]){
	rCoord[0] = x;
	rCoord[1] = y;
	rCoord[2] = phi;

	for (int i = 0; i < 6; ++i){
		for (int j = 0; j < 5; ++j){
			if(i < 1){
				distances[i][j] = sqrtf( powf( (rCoord[0]-newCoords[j][0]) , 2 ) + powf( (rCoord[1]-newCoords[j][1]) , 2 ) );
			}else if(j+1 == i){
			    distances[i][j] = 100;
			}else {
				distances[i][j] = sqrtf( powf( (newCoords[i-1][0]-newCoords[j][0]) , 2 ) + powf( (newCoords[i-1][1]-newCoords[j][1]) , 2 ) );
			}
		}
	}

	// print distance table
	cout << endl;
	for (int i = 0; i < 6; ++i){
		cout << i << endl;
		for (int j = 0; j < 5; ++j){
			cout << j << "  " << distances[i][j] << endl;
		}
		cout << endl;
	}

}


void choosePath(float distances[][5]){
	// This section determines the fastest path to each object location (so far only considering shortest distance)

	lowest = fabs(distances[0][0]);
    order[0] = 0;
    used[order[0]] = 1;
	for (int j = 1; j < 5; ++j){
        if (fabs(distances[0][j]) < lowest){
            lowest = fabs(distances[0][j]);
            used[order[0]] = 0;
            order[0] = j;
            used[order[0]] = 1;
        }
	}

	for (int k = 1; k < 5; ++k){
        lowest = fabs(distances[1][order[k-1]]);
        order[k] = 0;
        used[order[k]] = 1;
        for (int l = 1; l < 6; ++l){
            if (fabs(distances[l][order[k-1]]) < lowest && (order[k-1] + 1) != l && used[l-1] != 1){
                lowest = fabs(distances[l][order[k-1]]);
                used[order[k]] = 0;
                order[k] = l-1;
                used[order[k]] = 1;
            }
        }
	}

	cout << endl << "Order of Points to Visit" << endl;
	for (int i = 0; i < 5; ++i){
		cout << order[i] << ", ";
	}
	cout << endl << endl;

    cout << "Used Points" << endl;
	for (int i = 0; i < 5; ++i){
		cout << used[i] << ", ";
	}
	cout << endl << endl;

}

//-------------------------move robot function----------------
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

int main(int argc, char** argv){
	ros::init(argc, argv, "map_navigation_node");
	ros::NodeHandle n;
	ros::spinOnce();
  	teleController eStop;

	ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, poseCallback);

	vector<vector<float> > coord; // co-ordinate vector for boxes
	vector<cv::Mat> imgs_track;
	if(!init(coord, imgs_track)) return 0;

	for(int i = 0; i < coord.size(); ++i){
		cout << i << " x: " << coord[i][0] << " y: " << coord[i][1] << " z: " << coord[i][2] << endl;
	}

	imageTransporter imgTransport("/camera/rgb/image_raw", sensor_msgs::image_encodings::BGR8); // For Kinect

	Mat img_raisin = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/pics/tag1.jpg", IMREAD_GRAYSCALE );
	Mat img_cinnamon = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/pics/tag2.jpg", IMREAD_GRAYSCALE );
	Mat img_rice = imread("/home/turtlebot/catkin_ws/src/mie443_contest2/pics/tag3.jpg", IMREAD_GRAYSCALE );

	Mat img_cam =  imgTransport.getImg();



	// wait for coordinates to update
	while(x == 0 && y == 0){
		ros::spinOnce();
	}

	// rewrite desired distance and coords to spaces
	pathCalc(coord);
	// calculating distances and desired path
	distCalcs(newCoords);
	xStart = x;
	yStart = y;
	phiStart = phi;
	choosePath(distances);
	currLoc = 0;

	while(ros::ok()){
		ros::spinOnce();
  		//.....**E-STOP DO NOT TOUCH**.......
   		eStop.block();
    		//...................................
		if (currLoc <= 4){
			cout << endl << "Position " << currLoc + 1 << endl;
			success[currLoc] = moveToGoal(newCoords[order[currLoc]][0], newCoords[order[currLoc]][1], newCoords[order[currLoc]][2]);
							//moveToGoal(-1.5, 2.5, 0);
			ros::spinOnce();
			img_cam =  imgTransport.getImg();

			if(success[currLoc]){
				//cout << order[currLoc] << endl;
				//img_cam =  imgTransport.getImg();
				
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
			currLoc = currLoc + 1;
			

		} else if (currLoc == 5){
			for(int i=0;i<5;i++){
				if(!success[i]){
					success[i] = moveToGoal(newCoords[order[i]][0], newCoords[order[i]][1], newCoords[order[i]][2]); 
					ros::spinOnce();
					img_cam =  imgTransport.getImg();
					if(isMatch(img_raisin,img_cam)){
					  	tagAgain[i]=1;  
						cout << "This is a raisin." << endl;
					} 
					else if(isMatch(img_cinnamon,img_cam)){
					  	tagAgain[i]=2;
						cout << "This is a cinnamon." << endl;
					}
					else if(isMatch(img_rice,img_cam)){
						tagAgain[i]=3;
						cout << "This is a rice." << endl;
					} 
					else{
						tagAgain[i]=0;
					 	cout << "This is a blank." << endl;
					}
				}
			}
			moveToGoal(xStart, yStart, phiStart);
			cout << "Finished" << endl;
			currLoc = currLoc + 1;
			int saw[]={0,0,0,0,0};
			cout << "The tags are as follows:" << endl;
			for(int i=0;i<5;i++){
				cout << "Location " << i+1 << ": " ;
				switch(tag[i]){
					case 0 : cout << "Blank"; 
						if(saw[tag[i]]){
							cout << " (duplicate)" << endl;
						}
						else{
							cout << endl;
							saw[tag[i]]=1;
						} 		
						break;
					case 1 : cout << "Raisin Bran";
						if(saw[tag[i]]){
							cout << " (duplicate)" << endl;
						}
						else{
							cout << endl;
							saw[tag[i]]=1;
						} 
						break;
					case 2 : cout << "Cinnamon Toast Crunch"; 
						if(saw[tag[i]]){
							cout << " (duplicate)" << endl;
						}
						else{
							cout << endl;
							saw[tag[i]]=1;
						} 
						break;
					case 3 : cout << "Rice Krispies"; 
						if(saw[tag[i]]){
							cout << " (duplicate)" << endl;
						}
						else{
							cout << endl;
							saw[tag[i]]=1;
						} 
						break;
					default: cout << "Failed Scan" << endl;
				}

			}
			cout << endl;
			for(int i=0;i<5;i++){
				if(tagAgain[i]>-1){
					cout << "Location " << i+1 << " was visited again: " ;
					switch(tagAgain[i]){
						case 0 : cout << "Blank"; 
							if(saw[tagAgain[i]]){
								cout << " (duplicate)" << endl;
							}
							else{
								cout << endl;
								saw[tagAgain[i]]=1;
							} 		
							break;
						case 1 : cout << "Raisin Bran";
							if(saw[tagAgain[i]]){
								cout << " (duplicate)" << endl;
							}
							else{
								cout << endl;
								saw[tagAgain[i]]=1;
							} 
							break;
						case 2 : cout << "Cinnamon Toast Crunch"; 
							if(saw[tagAgain[i]]){
								cout << " (duplicate)" << endl;
							}
							else{
								cout << endl;
								saw[tagAgain[i]]=1;
							} 
							break;
						case 3 : cout << "Rice Krispies"; 
							if(saw[tagAgain[i]]){
								cout << " (duplicate)" << endl;
							}
							else{
								cout << endl;
								saw[tagAgain[i]]=1;
							} 
							break;
						default: cout << "Failed Scan" << endl;
					}
				}

			}
			return 1;
		}


		// final prints and etc go here


	}
	return 0; 
}


