
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ros/ros.h"
using namespace cv;
#define WINDOW_NAME "【滑动条示例】"        
const int g_nMaxValue = 254;//的最大值
int g_nValueSlider = 70;
Mat g_dstImage = Mat(500 , 600 , CV_8UC1);



void on_Trackbar(int , void*)
{
    g_dstImage.setTo(g_nValueSlider);
    imshow(WINDOW_NAME , g_dstImage);
}

int main(int argc , char** argv)
{
    ros::init(argc, argv, "bar_demo");
    namedWindow(WINDOW_NAME , 1);
    char TrackbarName[50];
    sprintf(TrackbarName , "灰度值");
    createTrackbar(TrackbarName , WINDOW_NAME ,
        &g_nValueSlider , g_nMaxValue , on_Trackbar);
    on_Trackbar(0 , 0);
    waitKey(0);

    return 0;
}