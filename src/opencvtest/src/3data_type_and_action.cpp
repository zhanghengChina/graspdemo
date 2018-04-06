//opencv练习3--操作不同数据类型，了解core模块，同时core组件还提供了很多在图像上画其他图形的函数

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "iostream"

using namespace std;
using namespace cv;
#define WINDOW_NAME "lenna"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "DataTypeAndAction");
    ros::NodeHandle nh;
    Mat image = imread("/home/leon/graspdemo/src/opencvtest/src/lenna.jpg");

    /*图像的不同复制方式*/
    Mat image_head_copy(image);
    //此时image_head_copy只是复制了image的信息头部分，两者指向的实际矩阵是一样的。
    //因此无论通过任何一个对象来操作，都会引起图像的改变
    namedWindow(WINDOW_NAME,WINDOW_AUTOSIZE);
    Mat image_clone = image.clone();
    Mat image_copy; 
    image.copyTo(image_copy);
    //通过clone和copy函数来对图像进行复制，此时不只是复制的矩阵头，还有实际的图像信息。
    imshow(WINDOW_NAME,image);
    waitKey();


    //opencv点的表示
    Point2f point2;
    point2.x = 1;
    point2.y = 2;
    Point3f point3(1,2,3);
    //opencv其他数据结构
    Scalar color(0,0,255);//设定颜色其实就是BGR
    Rect rect(10,10,100,100);//设定一个矩形，分别是左上角点坐标以及宽和高
    int width = rect.width;
    //Mat out_image;
    //cvtColor(image,out_image,COLOR_GRAY2BGR);//用来执行图像不同格式的转换
    //waitKey();
    
    return 0;
}