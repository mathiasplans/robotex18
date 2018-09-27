#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include "vision/Ball.h"
#include "vision/Threshold.h"
#include <boost/bind.hpp>

using namespace std;
using namespace cv;

Scalar upper;
Scalar lower;
Scalar houghArgs;
int lastx = 0;
int lasty = 0;

void detection_callback(const sensor_msgs::ImageConstPtr& ros_frame, image_transport::Publisher& pub, ros::Publisher& ballPub){
    //Convert ros image back to cv::Mat
    cv_bridge::CvImagePtr ptr;
    ptr = cv_bridge::toCvCopy(ros_frame, "bgr8");
    Mat frame = ptr->image;

    
    GaussianBlur( frame, frame, Size(3, 3), 2, 2 );
    
    Mat mask;
    cvtColor(frame, frame, CV_BGR2HSV);
    inRange(frame, lower, upper, mask);
    cvtColor(frame, frame, CV_HSV2BGR);
    cvtColor(frame, frame, CV_BGR2GRAY);
    bitwise_and(frame, mask, frame);

    vector<Vec3f> circles;
    HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 1, houghArgs[0], houghArgs[1], houghArgs[2], 3, 30); // Very experimental, more calibration needed
    

    cvtColor(frame, frame, CV_GRAY2BGR);
    
    for( int i = 0; i < circles.size(); i++ ){
        circle(frame, Point(cvRound(circles[i][0]), cvRound(circles[i][1])), cvRound(circles[i][2]), Scalar(0,0,255), 2);
    }

    ROS_INFO("Found %d balls", (int)circles.size());
    //---------------------------------
    
    // if(0 || largestIndex >= 0){
    //     vision::Ball ball;
        
    //     ball.ballX = center[largestIndex].x;
    //     ball.ballY = center[largestIndex].y;
    //     ball.width = 480;
    //     // ball.height = height;
    //     if(abs(ball.ballX - lastx) < 30 && abs(ball.ballY - lasty) < 30){
    //         ROS_INFO("Found %d balls", foundCount);
    //         ballPub.publish(ball);
    //     }

    //     lastx = center[largestIndex].x;
    //     lasty = center[largestIndex].y;
    // }

    

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    pub.publish(msg);

}

void threshold_callback(const vision::Threshold::ConstPtr& t){
    upper = Scalar(t->hh, t->sh, t->vh);
    lower = Scalar(t->hl, t->sl, t->vl);
    houghArgs = Scalar(t->dst, t->p1, t->p2);
    ofstream ths("/home/robot/catkin_ws/src/vision/ths.txt");
    ths << t->hh << endl << t->sh << endl << t->vh << endl;
    ths << t->hl << endl << t->sl << endl << t->vl << endl;
    ths << t->dst << endl << t->p1 << endl << t->p2 << endl;
    ths.close();
}

int main(int argc, char **argv){
    //Load previous thresholds from file
    int hh, sh, vh, hl, sl, vl, dst, p1, p2;
    ifstream ths("/home/robot/catkin_ws/src/vision/ths.txt");
    ths >> hh >> sh >> vh >> hl >> sl >> vl >> dst >> p1 >> p2;
    upper = Scalar(hh, sh, vh);
    lower = Scalar(hl, sl, vl);
    houghArgs = Scalar(dst, p1, p2);

    ros::init(argc, argv, "ball_detection");
    ros::NodeHandle n;
    
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("camera/processed", 1);
    ros::Publisher ballPub = n.advertise<vision::Ball>("ball", 50);
    ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("camera/stream", 1, boost::bind(detection_callback, _1, pub, ballPub));
    ros::Subscriber tsub = n.subscribe<vision::Threshold>("thresholds", 1, threshold_callback);
    
    

    ros::spin();
    while(ros::ok()){
    
    }

    return 0;
}