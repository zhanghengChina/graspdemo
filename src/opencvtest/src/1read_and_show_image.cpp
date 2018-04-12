//opencv练习1，在指定文件夹下面读取图像并且显示
// 同时使用canny算子来进行边缘检测

#include "ros/ros.h"
#include "opencv2/opencv.hpp"

using namespace cv;
#define WINDOW_NAME "lenna before border detection"
#define WINDOW_NAME1 "lenna after border detection"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ReadAndShowImage");
    ros::NodeHandle nh;
    Mat image = imread("/home/leon/graspdemo/src/opencvtest/src/lenna.jpg");
    Mat image1 = image.clone();
    namedWindow(WINDOW_NAME,WINDOW_AUTOSIZE);
    imshow(WINDOW_NAME,image);

    Mat dst, edge, gray;
    dst.create(image1.size(),image1.type());
    cvtColor(image1,gray,COLOR_BGR2GRAY);
    blur(gray,edge,Size(3,3));
    Canny(edge,edge,3,9,3);
    
    dst = Scalar::all(0);
    image1.copyTo(dst,edge);
    namedWindow(WINDOW_NAME1,WINDOW_AUTOSIZE);
    imshow(WINDOW_NAME1,dst);
    waitKey();
    return 0;
}