//opencv练习4，图像像素操作方法以及图像遍历

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "iostream"

using namespace std;
using namespace cv;
#define WINDOW_NAME "lenna"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ReadAndShowImage");
    ros::NodeHandle nh;
    Mat image = imread("/home/leon/graspdemo/src/opencvtest/src/lenna.jpg");
    namedWindow(WINDOW_NAME,WINDOW_AUTOSIZE);

    // 通过指针操作，这种方法最快
    // 要理解图像其实就是一个矩阵，存储在一块连续的空间里面，像操作数组那样操作矩阵
    int row = image.rows;
    int col = image.cols * image.channels();
    cout<<row<<" "<<col<<endl;
    for(int i = 0 ; i < row ; i ++)
    {
        uchar* data = image.ptr<uchar>(i);
        for(int j = 0 ; j < col; col ++)
        {
            data[j] = 0;
        }
    }    
    // 通过下标操作
    // 这种方法对于平时的操作也是比较简单
    int pixel = (int)image.at<Vec3b>(2,2)[1];
    cout<<pixel<<endl;
    imshow(WINDOW_NAME,image);
    waitKey();
    return 0;
}