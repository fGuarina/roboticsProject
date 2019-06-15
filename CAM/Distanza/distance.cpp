#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

// HSV parameters: max_value_H = 360/2 and max_value = 255; 

// ------------------- BALL --------------------- //

// HSV for green ball
//int low_H = 50, low_S = 116, low_V = 138;
//int high_H = 68, high_S = 210, high_V = 250;

// HSV for red ball
int low_H = 0, low_S = 118, low_V = 156;
int high_H = 10, high_S = 182, high_V = 255;


// ------------------- DOOR --------------------- //

// HSV for blue color port
//int low_H = 130, low_S = 60, low_V = 88;
//int high_H = 145, high_S = 135, high_V = 175;

// HSV for red door
//int low_H = 64, low_S = 131, low_V = 76;
//int high_H = 83, high_S = 255, high_V = 150;


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

// Returns the distance from the object. First have to calculate the distance and  corresponding  height
double distance (double h) {
	double dist;
	double distanceKnow = 33;	// pre calculated -> 50cm for DOOR and 33cm for ball
	double heightKnow = 148; 	
	
	return dist = ((heightKnow / 2) / h) * distanceKnow;
}

int main(int argc, char** argv) {
	VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
	
	// declares BGR frame, blurred frame, HSV frame and the final result of the threshold
	Mat frame, blurredFrame, frame_HSV, frame_threshold;
    
	while(true) {
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
  
		// Show the frames
        imshow("Original", frame);
        imshow("Thresholded Image", frame_threshold);
        char key = (char) waitKey(30);
        if (key == 'q' || key == 27)
        {
            break;
        }   
	}
	
	cap.release();
	
	// Detected contours. Each contour is stored as a vector of points
	vector <vector<Point>> contours;
	vector <Vec4i> hierarchy;		
	
	findContours(frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE); 	
	// see OpenCV doc at: https://docs.opencv.org/3.4/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a
	
	// Draw contours
	RNG rng(12345);
	Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255));
	Mat drawing = Mat::zeros(frame_threshold.size(), CV_8UC3);
	for(int i = 0; i< contours.size(); i++ ) {
		drawContours(drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
		//drawContours(drawing, contours, i, Scalar(0, 255, 0), CV_FILLED);			// fills the closed contours with color
	}
	
	imshow("All contours detected", drawing);
	waitKey();
	
	// contoursConvexHull call. Returns the closed countour that best approximates the object
	vector<Point> ConvexHullPoints =  contoursConvexHull(contours);
	
	// draw the contour
	polylines(drawing, ConvexHullPoints, true, Scalar(0,0,255), 2);
	imshow("Contour after convexHull usage (in red)", drawing);
	waitKey();
    
	// Bounding Box with rectangle
	Rect rect1;
	rect1 = boundingRect(ConvexHullPoints);
	
	rectangle(drawing, rect1, color, 2, 8, 0 );
	imshow("Boundig box on contour", drawing);
	waitKey();
	
	// distance calculation
	cout << "Height: " << rect1.height << endl; 		// it is used for precalculation
	double dist = distance(rect1.height/2);
		
	cout << "Distance from the object: " << dist << " cm" << endl;
	
	return 0;
}
