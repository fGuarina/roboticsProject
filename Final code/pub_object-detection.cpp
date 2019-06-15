
#include <iostream>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"

using namespace std;
using namespace cv; 

// initialization of the HSV parameters (max_value_H = 360/2 and max_value = 255)
int low_H = 0, low_S = 0, low_V = 0;
int high_H = 1, high_S = 1, high_V = 1;

// initialization of the height know 
float heightKnow = 0;	

void change_threshold_CB(const std_msgs::Int64::ConstPtr& msg){
	if(msg->data == 1){
		// Phase1
		
		// HSV parameters for our ball
		low_H = 0;
		high_H = 10;
		
		low_S = 118; 
		high_S = 182;
		
		low_V = 156;
		high_V = 255;
		
		// height know for our ball
		heightKnow = 163;
	}
	if(msg->data == 2){
		// Phase 2
		
		// HSV parameters for opponent's ball
		cout << "HSV param for phase 2!" << endl;
		low_H = 50;
		high_H = 68;
		
		low_S = 116;
		high_S = 210;
		
		low_V = 138;
		high_V = 255;
		
		// height know for opponent's ball
		heightKnow = 150;
	}
	if(msg->data == 3){
		// Phase 3
		
		// HSV parameters for opponent's DOOR
		cout << "HSV param for phase 3!" << endl;
		low_H = 64; 
		high_H = 83;
		
		low_S = 131; 
		high_S = 255;
		
		low_V = 76;
		high_V = 150;
		
		// height know for opponent's DOOR
		heightKnow = 284;
	}
	if(msg->data == 4){
		// Phase 4
		
		// HSV parameters for opponent's ball
		cout << "HSV param for phase 4!" << endl;
		low_H = 50;
		high_H = 68;
		
		low_S = 116;
		high_S = 210;
		
		low_V = 138;
		high_V = 255;
		
		// height know for opponent's ball
		heightKnow = 150;
	}
 }

// convexHull usage. This function returns the closed countour that best approximates the object
vector<Point> contoursConvexHull(vector <vector<Point>> contours)
{
    vector<Point> result;
    vector<Point> pts;
    
    for(size_t i = 0; i < contours.size(); i++)
        for(size_t j = 0; j < contours[i].size(); j++)
            pts.push_back(contours[i][j]);
            
    convexHull(pts, result );
    return result;
}

// Returns the distance from the object. First have to calculate the distance and corresponding height
float distance (double h) {
	float dist;
	float distanceKnow = 33;	// pre calculated
	
	return dist = ((heightKnow / 2) / h) * distanceKnow;
}


int main(int argc, char **argv) {
	// create the node
	ros::init(argc,argv,"Detection");
	
	// create a handle to this process' node. 
	ros::NodeHandle n;
	
	// instantiate the publishers
	ros::Publisher pub1 = n.advertise<std_msgs::Float64>("centroid_dist_channel", 1);
	ros::Publisher pub2 = n.advertise<std_msgs::Float64>("object_dist_channel", 1);
	ros::Publisher pub3 = n.advertise<std_msgs::Bool>("object_found_channel", 1);
	
	// instantiate the subscriber
	ros::Subscriber sub = n.subscribe("/phase_control_channel", 1, change_threshold_CB);
	
	// specify a frequency that you would like to loop at
	ros::Rate loop_rate(1.5);
	
	VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
	
	// declares BGR frame, HSV frame and the final result of the threshold
    Mat frame, frame_HSV, frame_threshold;
    
    // time needed to focus
    for(int i = 0; i <= 40; i++) 
			cap >> frame;
			
	while (ros::ok()) {
		cap >> frame;
		if(frame.empty()) {
            break;
        }
        
        // Convert from BGR to HSV colorspace 
		cvtColor(frame, frame_HSV, COLOR_BGR2HSV); 
        
		// Detect the object based on HSV Range Values
		inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
		
		// Morphological opening (removes small objects from the foreground)
		erode(frame_threshold, frame_threshold, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));
		dilate(frame_threshold, frame_threshold, getStructuringElement(MORPH_ELLIPSE, Size(7, 7))); 
		// see OpenCV doc at: https://docs.opencv.org/2.4/doc/tutorials/imgproc/erosion_dilatation/erosion_dilatation.html

		// Morphological closing (removes small holes from the foreground)
		dilate(frame_threshold, frame_threshold, getStructuringElement(MORPH_ELLIPSE, Size(7, 7))); 
		erode(frame_threshold, frame_threshold, getStructuringElement(MORPH_ELLIPSE, Size(7, 7)));	     
		
		// Detected contours. Each contour is stored as a vector of points
		vector <vector<Point>> contours;
		vector <Vec4i> hierarchy; 
	
		findContours(frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); 	
		// see OpenCV doc at: https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a
		
		// to prevent error from the convexHull function (bad threshold)
		if (contours.size() >= 1) {
			
			// contoursConvexHull call. Returns the closed countour that best approximates the object
			vector<Point> ConvexHullPoints =  contoursConvexHull(contours);
			
			double area = contourArea(ConvexHullPoints);
			
			// to avoid false positives
			if (area > 500) {
	
				// Bounding Box with rectangle
				Rect rect1;
				rect1 = boundingRect(ConvexHullPoints);
	
				// distance calculation
				float object_dist = distance(rect1.height/2);
				cout << "Object detected, Sir!" << endl;
	
				// Calculation of the centroid as division between moments 	
				vector<Moments> mu(ConvexHullPoints.size());
				for(int i = 0; i < ConvexHullPoints.size(); i++)
					mu[i] = moments(ConvexHullPoints, false); 
		 
				vector<Point2f> mc(ConvexHullPoints.size());
				for(int i = 0; i < ConvexHullPoints.size(); i++)
					mc[i] = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00); 
	
				float centroid_dist = mc[0].x - frame.cols/2;
			
				// publish the centroid distance
				std_msgs::Float64 msg1;
				msg1.data = centroid_dist;
				pub1.publish(msg1);
		
				// publish the distance from the object
				std_msgs::Float64 msg2;
				msg2.data = object_dist;
				pub2.publish(msg2);

				// info for the Search code
				std_msgs::Bool msg3;
				msg3.data = true;
				pub3.publish(msg3); 
				
			}
			else {
			cout << "False positive. No object detected!" << endl;
			std_msgs::Bool msg3;
			msg3.data = false;
			
			// info for the Search code
			pub3.publish(msg3); }
		}
		else {
		cout << "No object detected!" << endl;
		std_msgs::Bool msg3;
		msg3.data = false;
		
		// info for the Search code
		pub3.publish(msg3); }
			
		ros::spinOnce();
		
		// needed for the callback in subscriber's code
		loop_rate.sleep();
	}
	
	cap.release();
	
	return 0;
}
