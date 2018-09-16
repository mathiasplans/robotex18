#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main(int argc, char **argv){
    ros::init(argc, argv, "camera");

    ros::NodeHandle n;

    VideoCapture cap("/home/adrian/vid.mp4");
    

    if(!cap.isOpened()){
        cout << "Cannot open video\n";
        return -1;
    }

    double width = cap.get(CV_CAP_PROP_FRAME_WIDTH);
    double height = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
    

    cout << "Video opened, width: " << width << " and height: " << height << endl;

    namedWindow("TestVid", CV_WINDOW_AUTOSIZE);
    int frameCount = 0;

    while(1){

        Mat frame;
        bool fSuccess = cap.read(frame);

        if(!fSuccess){
            cout << "Cannot read frame nr " << frameCount << endl;
        }

        //imshow("TestVid", frame);

        frameCount++;
    }
    return 0;
}