/* This code is equal to the distance code (distance.cpp) but also implements the centroid calculation */

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace std;
using namespace cv;

// HSV parameters: max_value_H = 360/2 and max_value = 255; 

// HSV for green color
int low_H = 30, low_S = 40, low_V = 0;
int high_H = 90, high_S = 130, high_V = 125;

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
	double distanceKnow = 30;	// pre calculated
	double heightKnow = 131;	
	
	return dist = ((heightKnow / 2) / h) * distanceKnow;
}

int main(int argc, char** argv) {
	VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
	
	// declares BGR frame, blurred frame, HSV frame and the final result of the threshold
	Mat frame, frame_HSV, frame_threshold;
    
	while(true) {
		cap >> frame;	
		if(frame.empty()) {
            break;
        }
        
        // Image processing: blurring for a better extraction of the contours
	//	GaussianBlur(frame, blurredFrame, Size(7, 7), 0, 0);
        
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
	for(int i = 0; i < contours.size(); i++ ) {
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
	//cout << "Height: " << rect1.height << endl; 		// it is used for precalculation
	double dist = distance(rect1.height/2);
		
	cout << "Distance from the object: " << dist << " cm" << endl;
	
	
	// Calculation of the centroid as division between moments 	
	vector<Moments> mu(ConvexHullPoints.size());
	for(int i = 0; i < ConvexHullPoints.size(); i++)
		mu[i] = moments(ConvexHullPoints, false); 
		 
	vector<Point2f> mc(ConvexHullPoints.size());
	for(int i = 0; i < ConvexHullPoints.size(); i++)
		mc[i] = Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00); 
	
	// to draw the centroid
	for(int i = 0; i < ConvexHullPoints.size(); i++ )	
		circle(drawing, mc[i], 4, color, -1, 8, 0 );
		
	// show the centroid
	imshow("Centroid", drawing);
	waitKey(0);
	
	cout << "\nCentroid coordinates (x, y): " << mc[0] << endl;
	cout << "Frame size in pixel: " << frame.size() << endl;
	
	// distance between the centroid and the frame's center
	cout << "Distance between the centroid and the frame's center: " << mc[0].x - frame.cols/2 <<endl;
	
	cap.release();
	
	return 0;
}
