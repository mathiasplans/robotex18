#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include "vision/Ball.h"

// #define fake_camera false;
using namespace std;
using namespace cv;
const double pi = 3.14159265359;

int main(int argc, char **argv){
    ros::init(argc, argv, "camera");

    ros::NodeHandle n;
    ros::Publisher pub =  n.advertise<vision::Ball>("ball", 1000);
    

    ROS_INFO("Started");

    #ifdef fake_camera
    VideoCapture cap("/home/robot/vid2.mp4");
    #else
    VideoCapture cap(2);
    #endif

    if(!cap.isOpened()){
        cout << "Cannot open video\n";
        return -1;
    }

    double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);

    #ifdef fake_camera
    Size downscale;
    if(!(width > height)){
        downscale = Size(1280, 720);
    }else{
        downscale = Size(720, 1280);
    }
    #else
    Size downscale(480, 640); // Frame is 9:16 because camera is mounted horizontally
    #endif
    
    int codec = 0x00000021;///VideoWriter::fourcc('X', '2', '6', '4');
    double fps = 30.0;
    string filename = "/home/robot/vids/test1.mp4";
    VideoWriter out;

    out.open(filename, codec, fps, downscale, true);

    if(!out.isOpened()){
        cout << "Error could not open output file" << endl;
        return -1;
    }

    Scalar greenLower (29, 86, 6);
Scalar greenUpper(64, 255, 255);


    //namedWindow( "window", WINDOW_AUTOSIZE );
    

    cout << "Video opened, width: " << width << " and height: " << height << endl;

    //namedWindow("TestVid", CV_WINDOW_AUTOSIZE);
    int frameCount = 0;
    
    while(ros::ok()){

        Mat frame;
        
        bool fSuccess = cap.read(frame);

        if(!fSuccess){
            cout << "Cannot read frame nr " << frameCount << endl;
            return -1;
        }

        #ifndef fake_camera
        rotate(frame, frame, ROTATE_90_CLOCKWISE);
        #endif
        
        //Process frame
        Mat resized;
        resize(frame, frame, downscale);

        //GaussianBlur(frame, frame, Size(11, 11), 0);
	    cvtColor(frame, frame, COLOR_BGR2HSV);

        Mat element = getStructuringElement( MORPH_RECT,
                       Size(3, 3),
                       Point(1, 1));
        Mat mask;
        inRange(frame, greenLower, greenUpper, mask);

        
        erode(mask, mask, element, Point(-1, -1),2);
        dilate(mask, mask, element, Point(-1, -1),2);

        
        //std::vector<std::vector<cv::Point> > contours;
        std::vector<std::vector<cv::Point> > contours;
        //std::vector<cv::Vec4i> hierarchy
        findContours(mask, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        vector<vector<Point> > contours_poly( contours.size() );
        //vector<Rect> boundRect( contours.size() );
        vector<Point2f>center( contours.size() );
        vector<float>radius( contours.size() );

        for( int i = 0; i < contours.size(); i++ ){
            approxPolyDP( Mat(contours[i]), contours_poly[i], 1, true );
            //boundRect[i] = boundingRect( Mat(contours_poly[i]) );
            minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
            }

        cvtColor(frame, frame, CV_HSV2BGR);

        int largestIndex = -1;
        float largestRadius = 0;
        int foundCount = 0;
        
        for( int i = 0; i< contours.size(); i++ ){
            // Filter off too small and non-circular contours
            // if(radius[i] < 10) continue;
            if(contourArea(contours[i]) < pi * radius[i] * radius[i] * 0.4) continue;
            foundCount++;
            if(radius[i] > largestRadius){
                largestIndex = i;
                largestRadius = radius[i];
            }
            drawContours(frame, contours_poly, i, Scalar(0, 0, 255), 2, 8, vector<Vec4i>(), 0, Point() );
            circle(frame, center[i], 4, Scalar(0, 0, 0), 2, 8, 0 );
            circle(frame, center[i], radius[i], Scalar(0,0,0), 1);
            }

        if(largestIndex >= 0){
            vision::Ball ball;
            ball.ballX = center[largestIndex].x;
            ball.ballY = center[largestIndex].y;
            ball.width = width;
            ball.height = height;
            ROS_INFO("Found %d balls", foundCount);
            pub.publish(ball);
        }
        // ROS_INFO("Frame processed"); 
        // cvtColor(mask, mask, CV_GRAY2BGR);
        out.write(frame);

        // resize(frame, frame, Size(240, 426));
        //imshow("window", frame);
        
        // ROS_INFO("Frame displayed");
        // waitKey(300);

        // if(frameCount == 200){
        //     return 0;
        // }
        frameCount++;

    }
    return 0;
}
