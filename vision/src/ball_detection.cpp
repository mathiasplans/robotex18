#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include "vision/Ball.h"

using namespace std;
using namespace cv;

void detection_callback(const sensor_msgs::ImageConstPtr& ros_frame){
    //Convert ros image back to cv::Mat
    cv_bridge::CvImagePtr ptr;
    ptr = cv_bridge::toCvCopy(ros_frame, "bgr8");
    Mat frame = ptr->image;

    GaussianBlur(frame, frame, Size(11, 11), 0);
    cvtColor(frame, frame, COLOR_RGB2HSV);

    Mat element = getStructuringElement( MORPH_RECT,
                    Size(3, 3),
                    Point(1, 1));
    Mat mask;
    Scalar greenLower(36, 116, 96);
    Scalar greenUpper(66, 146, 126);
    inRange(frame, greenLower, greenUpper, mask);

    
    erode(mask, mask, element, Point(-1, -1),2);
    dilate(mask, mask, element, Point(-1, -1),2);

    std::vector<std::vector<cv::Point> > contours;
    findContours(mask, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Point2f>center( contours.size() );
    vector<float>radius( contours.size() );

    for( int i = 0; i < contours.size(); i++ ){
        approxPolyDP( Mat(contours[i]), contours_poly[i], 1, true );
        minEnclosingCircle( (Mat)contours_poly[i], center[i], radius[i] );
        }

    cvtColor(frame, frame, CV_HSV2RGB);

    int largestIndex = -1;
    float largestRadius = 0;
    int foundCount = 0;
    
    for( int i = 0; i< contours.size(); i++ ){
        // if(radius[i] < 4) continue;
        // if(contourArea(contours[i]) < pi * radius[i] * radius[i] * 0.6) continue;
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
        // ball.width = width;
        // ball.height = height;
        ROS_INFO("Found %d balls", foundCount);
        //pub.publish(ball);
    }

}

int main(int argc, char **argv){
    ros::init(argc, argv, "ball_detection");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("camera/stream", 1, detection_callback);

    ros::spin();
    while(ros::ok()){

    }

    return 0;
}