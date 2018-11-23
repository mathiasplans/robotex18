#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "ball_detection.hpp"
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include "vision/Ball.h"
#include "vision/Threshold.h"
#include "vision/Output_type.h"
#include "vision/BasketRelative.h"
#include <boost/bind.hpp>
#include <cmath>
#include "core/Bob.h"
#include <utility> 
#include <string>

#define PI 3.14159265359f

using namespace std;
using namespace cv;

bool dArr = false;
bool cArr = false;
pair<Mat,  int32_t> color;
pair<Mat,  int32_t> depth;
vector<pair<int, int> > lastBalls;
Scalar upperBall;
Scalar lowerBall;
Scalar upperBasket;
Scalar lowerBasket;
output_t output_type = DEF;
basket_t basket_type = PINK;
bool display_contours = false;
Point mouse(0,0);
int frameCount = 0;
vector<int> vals(15, 0);
int valcounter = 0;


void depth_callback(const sensor_msgs::ImageConstPtr& ros_frame){
    dArr = true;
    cv_bridge::CvImagePtr ptr;
    ptr = cv_bridge::toCvCopy(ros_frame, "");
    depth.first = ptr->image;
    std_msgs::Header h = ros_frame->header;
    depth.second = h.stamp.nsec;
    // cout << 1 << " " << depth.second << endl;
}

void color_callback(const sensor_msgs::ImageConstPtr& ros_frame){
    cArr = true;
    cv_bridge::CvImagePtr ptr;
    ptr = cv_bridge::toCvCopy(ros_frame, "");
    color.first = ptr->image;
    std_msgs::Header h = ros_frame->header;
    color.second = h.stamp.nsec;
    // cout << 0 << " " << color.second << endl;
}

bool ball_checker(){
    int len = lastBalls.size();
    if(len == 0){
        return false;
    }

    pair<int, int> lastBall = lastBalls[len - 1];

    int matching_len = 0;

    for(int i = len - 2; i >= 0; i--){
        if(abs(lastBalls[i].first - lastBall.first) < 100 && abs(lastBalls[i].second - lastBall.second) < 40){
            matching_len++;
        }
    }

    if(matching_len > 3){
        return true;
    }

    return false;
    
}

void ball_detection(image_transport::Publisher& pub, ros::Publisher& ballPub){
    
    Mat frame = color.first;
    Mat dframe = depth.first;
    
    rotate(frame, frame, ROTATE_90_CLOCKWISE);

    GaussianBlur(frame, frame, Size(3, 3), 0);
    cvtColor(frame, frame, COLOR_RGB2HSV);

    Mat element = getStructuringElement( MORPH_RECT,
                    Size(3, 3),
                    Point(1, 1));
    Mat mask;

    inRange(frame, lowerBall, upperBall, mask);
    
    // erode(mask, mask, element, Point(-1, -1),2);
    // dilate(mask, mask, element, Point(-1, -1),2);

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
        if(radius[i] < 3 ) continue;//|| radius[i] > 55 || center[i].y < 100) continue;
        if(polsby_doppler(contours[i]) < 0.65) continue;
        foundCount++;
        if(radius[i] > largestRadius){
            largestIndex = i;
            largestRadius = radius[i];
        }
        if(display_contours) drawContours(frame, contours_poly, i, Scalar(0, 0, 255), 2, 8, vector<Vec4i>(), 0, Point() );
        
    }

    vision::Ball ball;
    
    if(largestIndex >= 0){

        
        
        lastBalls.push_back(pair<int, int>(center[largestIndex].x, center[largestIndex].y));
        // abs(center[largestIndex].x - lastx) < 30 && abs(center[largestIndex].y - lasty) < 30
        if(ball_checker()){
            // ROS_INFO("Found %d balls", largestIndex);
            
            ball.ballX = center[largestIndex].x;
            ball.ballY = center[largestIndex].y;
            // TODO: Change these to a constant which the core can read. They are frame width and height.
            ball.width = 480;
            ball.height = 640;
        }else{
            ball.ballX = -1;
            ball.ballY = -1;
            ball.width = -1;
            ball.height = -1;
        }

        if(lastBalls.size() > 6){
            lastBalls.erase(lastBalls.begin());
        }
    }else{
        ball.ballX = -1;
        ball.ballY = -1;
        ball.width = -1;
        ball.height = -1;
    }

    // requested = false;
    ballPub.publish(ball);

    Mat out;

    switch(output_type){
        case DEF:
            out = frame;
            break;
        case MASK:
            // cvtColor(frame, frame, CV_BGR2GRAY);
            cvtColor(mask, mask, CV_GRAY2RGB);
            bitwise_and(frame, mask, frame);
            // bitwise_and(frame, frame, frame);
            // cvtColor(frame, frame, CV_GRAY2BGR);
            out = frame;
            break;
    }

    if(display_contours && largestIndex >= 0){
        circle(out, center[largestIndex], 4, Scalar(0, 0, 0), 2, 8, 0 );
        circle(out, center[largestIndex], largestRadius, Scalar(0,0,255), 1);
    }

    resize(frame, frame, Size(480, 640));

    frame = frame * 3;

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", frame).toImageMsg();
    pub.publish(msg);

}

struct Mod{
  void operator()(u_char &f, const int *p) const {
    f % *p;
  }
}; 

void basket_detection(image_transport::Publisher& pub, ros::Publisher& basketPub, ros::Publisher& basketRelPub){
    Mat cframe = color.first;
    Mat dframe = depth.first;

    rotate(cframe, cframe, ROTATE_90_CLOCKWISE);
    // rotate(dframe, dframe, ROTATE_90_CLOCKWISE);

    // cout << "Color frame height " << cframe.size().height << ", color frame width " << cframe.size().width << endl;
    // cout << "Depth frame height " << dframe.size().height << ", depth frame width " << dframe.size().width << endl;

    // TODO: reconsider blurring the image as the depth frame isn't blurred
    GaussianBlur(cframe, cframe, Size(7, 7), 0);
    cvtColor(cframe, cframe, COLOR_RGB2HSV);

    Mat element = getStructuringElement( MORPH_RECT,
                    Size(3, 3),
                    Point(1, 1));
    Mat mask;

    

    Mat hue = cframe.clone();

    multiply(hue, Scalar(1, 0, 0), hue);

    // hue += Scalar(30, 0, 0);
    // // Hacked together modulo operation on mat
    hue.forEach<u_char>(Mod());

    multiply(cframe, Scalar(0, 1, 1), cframe);

    cframe += hue;


    inRange(cframe, lowerBasket, upperBasket, mask);
    
    erode(mask, mask, element, Point(-1, -1),2);
    dilate(mask, mask, element, Point(-1, -1),2);

    std::vector<std::vector<cv::Point> > contours;
    findContours(mask, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    vector<vector<Point> > contours_poly( contours.size() );
    vector<Rect> boundRect( contours.size() );

    for( int i = 0; i < contours.size(); i++ ){
        approxPolyDP( Mat(contours[i]), contours_poly[i], 1, true );
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
        }

    cvtColor(cframe, cframe, CV_HSV2RGB);

    int largestIndex = -1;
    float largestArea = 0;
    int foundCount = 0;
    
    for( int i = 0; i< contours.size(); i++ ){
        // if(boundRect[i].width > boundRect[i].height * 0.4) continue;
        foundCount++;
        
        int area = boundRect[i].width * boundRect[i].height;

        if(area < 20 * 40) continue;

        if(area > largestArea){
            largestIndex = i;
            largestArea = area;
        }
        
    }


    Mat out;

    

    switch(output_type){
        case DEF:
            out = cframe;
            break;
        case MASK:
            // cvtColor(cframe, cframe, CV_RGB2GRAY);
            // bitwise_and(cframe, mask, cframe);
            // cvtColor(cframe, cframe, CV_GRAY2RGB);
            cvtColor(mask, mask, CV_GRAY2RGB);
            bitwise_and(cframe, mask, cframe);
            out = cframe;
            break;
    }

    vision::Ball ball;

    int depth;

    if(largestIndex >= 0){
        Rect r = boundRect[largestIndex];
        // uint16_t a = avg(dframe, Rect(Point(mouse.x - 5, mouse.y - 5), Point(mouse.x + 5, mouse.y + 5))); // Mouse testing
        uint16_t a = avg(dframe, r);
        int sum = 0;
        for(int i = 0; i < vals.size(); i++){
            sum += vals[i];
        }
        // putText(out, to_string(a), Point(mouse.x + 5, mouse.y + 5), FONT_HERSHEY_SIMPLEX, 1, Scalar(0,0,0), 2); // Mouse testing
        depth = sum/vals.size();

        
        vision::BasketRelative basket;
        basket.depth = depth;
        basketRelPub.publish(basket);

        putText(out, to_string(depth), Size(130, 640 - 100), FONT_HERSHEY_SIMPLEX, 3, Scalar(255,0,0), 5);
        
        drawContours(out, contours, largestIndex, Scalar(255, 0, 0));
        rectangle(out, r.tl(), r.br(), Scalar(0,0,255), 1, 8, 0); // Mouse testing
    }
    
    if(largestIndex >= 0 && depth < 4200){
        Rect r = boundRect[largestIndex];

        
        ball.ballX = r.x;
        ball.ballY = r.y;
        ball.width = r.width;
        ball.height = r.height;

    }else{
        ball.ballX = -1;
        ball.ballY = -1;
        ball.width = -1;
        ball.height = -1;
    }

    basketPub.publish(ball);
    cframe = cframe * 3;
    
    resize(cframe, cframe, Size(480, 640));

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", cframe).toImageMsg();
    pub.publish(msg);

    // if(display_contours && largestIndex >= 0){
    //     exit(0);
    // }

}

// --------------------------------------------
void threshold_callback_basket(const vision::Threshold::ConstPtr& t){
    upperBasket = Scalar(t->hh, t->sh, t->vh);
    lowerBasket = Scalar(t->hl, t->sl, t->vl);
    ofstream ths;
    if(basket_type == PINK){
        ths = ofstream("/home/robot/catkin_ws/src/vision/ths_basket_pink.txt");
    }else{
        ths = ofstream("/home/robot/catkin_ws/src/vision/ths_basket_blue.txt");
    }
    ths << t->hh << endl << t->sh << endl << t->vh << endl;
    ths << t->hl << endl << t->sl << endl << t->vl << endl;
    ths.close();
}
// --------------------------------------------

void threshold_callback_ball(const vision::Threshold::ConstPtr& t){
    upperBall = Scalar(t->hh, t->sh, t->vh);
    lowerBall = Scalar(t->hl, t->sl, t->vl);
    ofstream ths("/home/robot/catkin_ws/src/vision/ths.txt");
    ths << t->hh << endl << t->sh << endl << t->vh << endl;
    ths << t->hl << endl << t->sl << endl << t->vl << endl;
    ths.close();
}

// --------------------------------------------
void output_callback(const vision::Output_type::ConstPtr& o){
    if(o->t ==  "contours"){
        display_contours = !display_contours;
    }else if(o->t == "mask"){
        output_type = MASK;
    }else{
        output_type = DEF;
    }
}

void basket_type_callback(const std_msgs::String::ConstPtr& t, ros::Publisher& pub){
    cout << "Switch basket to color " << t->data << endl;
    if(t->data == "pink"){
        basket_type = PINK;
    }else if(t->data == "blue"){
        basket_type = BLUE;
    }else{
        cout << "ERROR WRONG BASKET COLOR: " << t->data << endl;
        return;
    }

    init_thresholds();

    vision::Threshold th;

    th.hh = upperBasket[0];
    th.sh = upperBasket[1];
    th.vh = upperBasket[2];
    th.hl = lowerBasket[0];
    th.sl = lowerBasket[1];
    th.vl = lowerBasket[2];

    pub.publish(th);

    
}

double polsby_doppler(std::vector<cv::Point>& contour){
    double length = arcLength(contour, true);
    double area = contourArea(contour);

    return 4*PI*area/pow(length, 2.0);
}

uint16_t avg(Mat& depths, Rect bounding){
    uint32_t sum = 0;
    uint32_t count = 0;

    Point tl = bounding.tl(); // (Min x, min y)
    Point br = bounding.br(); // (Max x, max y)

    // cout << "Top left - " << tl.x << " : " << tl.y << endl;
    // cout << "Bottom right - " << br.x << " : " << br.y << endl;

    
    for (int x = tl.x + 10; x <= br.x - 10; x++){
        for(int y = tl.y + 10; y <= br.y - 10; y++){
            uint16_t val = depths.at<uint16_t>(480 - x, y );
            if(val > 100){
                sum += val;
                count++;
            }
        }
        
    }

    // cout << (double)sum / (double)count << endl;
    // cout << "Count: " << count << endl;
    // exit(0);
    vals[valcounter] = (int)  ((double)sum / (double)count);
    valcounter = (valcounter + 1) % vals.size();
    return (uint16_t)  ((double)sum / (double)count);

}

void mouse_callback(const vision::Threshold::ConstPtr& t){
    mouse.x = t->hh;
    mouse.y = t->sh;
    cout << mouse.x << " : " << mouse.y << endl;
}

void init_thresholds(){
    // Ball
    int hh, sh, vh, hl, sl, vl;
    ifstream ths_ball("/home/robot/catkin_ws/src/vision/ths.txt");
    ths_ball >> hh >> sh >> vh >> hl >> sl >> vl;
    upperBall = Scalar(hh, sh, vh);
    lowerBall = Scalar(hl, sl, vl);
    
    // Basket
    ifstream ths_basket;
    if(basket_type == PINK){
        ths_basket = ifstream("/home/robot/catkin_ws/src/vision/ths_basket_pink.txt");
    }else{
        ths_basket = ifstream("/home/robot/catkin_ws/src/vision/ths_basket_blue.txt");
    }
    ths_basket >> hh >> sh >> vh >> hl >> sl >> vl;
    upperBasket = Scalar(hh, sh, vh);
    lowerBasket = Scalar(hl, sl, vl);
}



// Main
// --------------------------------------------
int main(int argc, char **argv){

    //Load previous thresholds from file
    init_thresholds();

    ros::init(argc, argv, "ball_detection");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    image_transport::Publisher basket_pub = it.advertise("camera/processed_basket", 1);
    image_transport::Publisher ball_pub = it.advertise("camera/processed_ball", 1);
    ros::Publisher threshold_pub = n.advertise<vision::Threshold>("basket_thresholds", 1);
    ros::Subscriber basket_threshold_sub = n.subscribe<vision::Threshold>("thresholds/basket", 1, threshold_callback_basket);
    ros::Subscriber ball_threshold_sub = n.subscribe<vision::Threshold>("thresholds/ball", 1, threshold_callback_ball);
    ros::Subscriber osub = n.subscribe<vision::Output_type>("output_type", 1, output_callback);
    ros::Subscriber dsub = n.subscribe<sensor_msgs::Image>("camera/aligned_depth_to_color/image_raw", 1, depth_callback);
    ros::Subscriber sub = n.subscribe<sensor_msgs::Image>("camera/color/image_raw", 1, color_callback);
    ros::Subscriber msub = n.subscribe<vision::Threshold>("mousexy", 1, mouse_callback);
    ros::Subscriber typesub = n.subscribe<std_msgs::String>("basket_type", 1, boost::bind(basket_type_callback, _1, boost::ref(threshold_pub)));

    ros::Publisher ballPub = n.advertise<vision::Ball>("ball", 1);
    ros::Publisher basketPub = n.advertise<vision::Ball>("basket", 1);
    ros::Publisher basketRelPub = n.advertise<vision::BasketRelative>("basketrelative", 1);
    
    
    // ros::spin();
    while(ros::ok()){
        
        ros::spinOnce();

        if(dArr & cArr){
            // dArr = false;
            // cArr = false;

            if(color.second == depth.second){
                dArr = false;
                cArr = false;
                frameCount++;
                // cout << "---------------------------------------------------\n";
                basket_detection(basket_pub, basketPub, basketRelPub);

                ball_detection(ball_pub, ballPub);

                // cout << color.second << endl;
            }else{
                // cout << "Error!" << endl;
                // break;
                //Frames weren't in sync, handler todo
                
            }
            
        }

    }

    return 0;
}
