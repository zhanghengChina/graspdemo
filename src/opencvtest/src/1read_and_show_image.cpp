//opencv练习1，在指定文件夹下面读取图像并且显示

#include "ros/ros.h"
#include "opencv2/opencv.hpp"

using namespace cv;
#define WINDOW_NAME "lenna"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ReadAndShowImage");
    ros::NodeHandle nh;
    Mat image = imread("/home/leon/graspdemo/src/opencvtest/src/lenna.jpg");
    namedWindow(WINDOW_NAME,WINDOW_AUTOSIZE);
    ROS_INFO_STREAM("The depth of the image is "<<image.depth());
    imshow(WINDOW_NAME,image);
    waitKey();
    return 0;
}