#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "ball_detection.hpp"
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include "vision/Ball.h"
#include "vision/Threshold.h"
#include "vision/Output_type.h"
#include "core/Bob.h"
#include <boost/bind.hpp>
#include <cmath>

#define PI 3.14159265359f

using namespace std;
using namespace cv;

Scalar upper;
Scalar lower;
int lastx = 0;
int lasty = 0;
output_t type = DEF;
bool display_contours = false;
bool requested = true;
int frameCount = 0;

double polsby_doppler(std::vector<cv::Point>& contour){
    double length = arcLength(contour, true);
    double area = contourArea(contour);

    return 4*PI*area/pow(length, 2.0);
}

void detection_callback(const sensor_msgs::ImageConstPtr& ros_frame, image_transport::Publisher& pub, ros::Publisher& ballPub){
    frameCount++;
    if(!requested) return;
    //Convert ros image back to cv::Mat
    cv_bridge::CvImagePtr ptr;
    ptr = cv_bridge::toCvCopy(ros_frame, "bgr8");
    Mat frame = ptr->image;

    GaussianBlur(frame, frame, Size(7, 7), 0);
    cvtColor(frame, frame, COLOR_BGR2HSV);

    Mat element = getStructuringElement( MORPH_RECT,
                    Size(3, 3),
                    Point(1, 1));
    Mat mask;

    inRange(frame, lower, upper, mask);
    
    // erode(mask, mask, element, Point(-1, -1),2);
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

    cvtColor(frame, frame, CV_HSV2BGR);

    int largestIndex = -1;
    float largestRadius = 0;
    int foundCount = 0;
    
    for( int i = 0; i< contours.size(); i++ ){
        if(radius[i] < 3 || radius[i] > 55 || center[i].y < 100) continue;
        if(polsby_doppler(contours[i]) < 0.75) continue;
        foundCount++;
        if(radius[i] > largestRadius){
            largestIndex = i;
            largestRadius = radius[i];
        }
        // if(display_contours) drawContours(frame, contours_poly, i, Scalar(0, 0, 255), 2, 8, vector<Vec4i>(), 0, Point() );
        
    }

    vision::Ball ball;
    
    if(largestIndex >= 0 || frameCount < 45){
        
        
        if(abs(center[largestIndex].x - lastx) < 30 && abs(center[largestIndex].y - lasty) < 30){
            // ROS_INFO("Found %d balls", largestIndex);
            
            ball.ballX = center[largestIndex].x;
            ball.ballY = center[largestIndex].y;
            ball.width = 480;
            ball.height = 640;
        }else{
            ball.ballX = -1;
            ball.ballY = -1;
            ball.width = -1;
            ball.height = -1;
        }

        lastx = center[largestIndex].x;
        lasty = center[largestIndex].y;
    }else{
        ball.ballX = -1;
        ball.ballY = -1;
        ball.width = -1;
        ball.height = -1;
    }

    // requested = false;
    ballPub.publish(ball);

    Mat out;

    

    switch(type){
        case DEF:
            out = frame;
            break;
        case MASK:
            cvtColor(frame, frame, CV_BGR2GRAY);
            bitwise_and(frame, mask, frame);
            cvtColor(frame, frame, CV_GRAY2BGR);
            out = frame;
            break;
    }

    if(display_contours && largestIndex >= 0){
        circle(out, center[largestIndex], 4, Scalar(0, 0, 0), 2, 8, 0 );
        circle(out, center[largestIndex], largestRadius, Scalar(0,0,255), 1);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);

}

void threshold_callback(const vision::Threshold::ConstPtr& t){
    upper = Scalar(t->hh, t->sh, t->vh);
    lower = Scalar(t->hl, t->sl, t->vl);
    ofstream ths("/home/robot/catkin_ws/src/vision/ths.txt");
    ths << t->hh << endl << t->sh << endl << t->vh << endl;
    ths << t->hl << endl << t->sl << endl << t->vl << endl;
    cout << "hi\n";
    ths.close();
}

void bob_callback(const core::Bob::ConstPtr& t){
    if(t->ball){
        requested = true;
    }
}

void output_callback(const vision::Output_type::ConstPtr& o){
    if(o->t ==  "contours"){
        display_contours = !display_contours;
    }else if(o->t == "mask"){
        type = MASK;
    }else{
        type = DEF;
    }
}

int main(int argc, char **argv){
    //Load previous thresholds from file
    int hh, sh, vh, hl, sl, vl;
    ifstream ths("/home/robot/catkin_ws/src/vision/ths.txt");
    ths >> hh >> sh >> vh >> hl >> sl >> vl;
    upper = Scalar(hh, sh, vh);
    lower = Scalar(hl, sl, vl);

    ros::init(argc, argv, "ball_detection");
    ros::NodeHandle n;
    
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("camera/processed_ball", 1);
    ros::Publisher ballPub = n.advertise<vision::Ball>("ball", 50);
    ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("camera/stream", 1, boost::bind(detection_callback, _1, pub, ballPub));
    ros::Subscriber tsub = n.subscribe<vision::Threshold>("thresholds/ball", 1, threshold_callback);
    ros::Subscriber osub = n.subscribe<vision::Output_type>("output_type", 1, output_callback);
    ros::Subscriber csub = n.subscribe<core::Bob>("bob", 10, bob_callback);
    

    ros::spin();
    while(ros::ok()){
    
    }

    return 0;
}