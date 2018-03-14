#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <func_header.h>

#include <eStop.h>
#include <math.h>

#define PI 3.14159265

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

float x;
float y;
float phi;

//variables for finding distances between pointss
int nB = 5;
float distances[6][5] = {0};
float rCoord[3];
int order[5];
bool used[5] = {0};
float lowest;
int cnt = 0;
float newCoords [5][3];
float botDistance = 0.3;


void poseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg){
	phi = tf::getYaw(msg.pose.pose.orientation);
    	x = msg.pose.pose.position.x;
    	y = msg.pose.pose.position.y;
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
	for (i = 0; i < 5; ++i){
		cout << order[i] << ", ";
	}
	cout << endl << endl;

    cout << "Used Points" << endl;
	for (i = 0; i < 5; ++i){
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

	// rewrite desired distance and coords to spaces
	pathCalc(coord);
	// calculating distances and desired path
	distCalcs(newCoords);
	choosePath(distances);
	cnt = 1;

	while(ros::ok()){
		ros::spinOnce();
  		//.....**E-STOP DO NOT TOUCH**.......
   		eStop.block();
    		//...................................

		if (cnt <= 4){
			cout << endl << "Position " << cnt << endl;
			moveToGoal(newCoords[order[cnt]][0], newCoords[order[cnt]][1], newCoords[order[cnt]][2]);
			//moveToGoal(-1.5, 2.5, 0);
			cnt = cnt + 1;
			cout << order[cnt] << endl;

			// image recognition stuff goes here or can be changed, depends if you need the while loop or not

		}

		// final prints and etc go here


	}
	return 0;
}


