//opencv练习2--在图像创建滑动条，可以通过滑动条来传递参数来处理图像
//目前回调函数还没有实质的处理内容

#include "ros/ros.h"
#include "opencv2/opencv.hpp"


using namespace cv;
#define WINDOW_NAME "lenna"
const int MaxValue = 100;
int current_value = 10;
Mat image;
void trackbarHandle(int,void*)
{
    imshow(WINDOW_NAME,image);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "TracBarCreator");
    ros::NodeHandle nh;
    namedWindow(WINDOW_NAME,WINDOW_AUTOSIZE);
    image = imread("/home/leon/graspdemo/src/opencvtest/src/lenna.jpg");
    createTrackbar("透明度100",WINDOW_NAME,&current_value,MaxValue,trackbarHandle);
    trackbarHandle(current_value,0);
    ROS_INFO("You are in the first line");
    //到这里的时候会进入回调函数的调用
    waitKey(0);
    return 0;
}