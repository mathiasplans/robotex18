#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include "vision/Break.h"

#define fake_camera
// #define vid_out

using namespace std;
using namespace cv;
const double pi = 3.14159265359;
bool stop = false;

void break_callback(const vision::Break::ConstPtr& s){
    stop = !stop;
    ROS_INFO("Break requested");
}

int main(int argc, char **argv){
    ros::init(argc, argv, "camera");

    ros::NodeHandle n;
    //ros::Publisher pub =  n.advertise<vision::Ball>("ball", 1000);
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("camera/stream", 1);
    ros::Subscriber bsub = n.subscribe<vision::Break>("break", 1, break_callback);

    ROS_INFO("Started");

    #ifdef fake_camera
    VideoCapture cap("/home/robot/vids/sample_footage.mp4");
    // VideoCapture cap("/home/robot/vid.mp4");
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
    downscale = Size(480, 640);
    ros::Rate loop_rate(30);
    #else
    Size downscale(480, 640); // Frame is 9:16 because camera is mounted horizontally
    #endif
    
    #ifdef vid_out
    int codec = 0x00000021;///VideoWriter::fourcc('X', '2', '6', '4');
    double fps = 30.0;
    string filename = "/home/robot/vids/test1.mp4";
    VideoWriter out;

    out.open(filename, codec, fps, downscale, true);

    if(!out.isOpened()){
        cout << "Error could not open output file" << endl;
        return -1;
    }
    #endif

    cout << "Video opened, width: " << width << " and height: " << height << endl;

    int frameCount = 0;
    Mat breakFrame;
    
    while(ros::ok()){

        Mat frame;

        if(!stop){
            bool fSuccess = cap.read(frame);
            if(!fSuccess){
                cout << "Cannot read frame nr " << frameCount << endl;
                return -1;
            }
            breakFrame = frame;
        }else{
            frame = breakFrame;
        }
        
        

        #ifndef fake_camera
        rotate(frame, frame, ROTATE_90_CLOCKWISE);
        #endif
        
        resize(frame, frame, downscale);

        #ifdef vid_out
        out.write(frame);
        #endif

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        pub.publish(msg);
        ros::spinOnce();

        #ifdef fake_camera
        loop_rate.sleep();
        #endif
        
        frameCount++;

    }
    return 0;
}
