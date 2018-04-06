// opencv练习5--综合滑动条+图像加权的实例

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "iostream"

using namespace std;
using namespace cv;
#define WINDOW_NAME "lenna"
const int MaxValue = 100;
int current_value;
Mat image1 = imread("/home/leon/graspdemo/src/opencvtest/src/lenna.jpg");
Mat image2 = imread("/home/leon/graspdemo/src/opencvtest/src/harden.jpg");
Mat mask = imread("/home/leon/graspdemo/src/opencvtest/src/lenna.jpg",0);
Mat imageRoi = image2(Rect(200,50,image1.cols,image1.rows));

bool roi_blender()
{
    image1.copyTo(imageRoi,mask);
    imshow(WINDOW_NAME,image2);
    waitKey();
    return 1;
}
bool addweight_blender(double alpha)
{
    alpha = alpha/100;
    cout<<alpha<<endl;
    addWeighted(image1,alpha,imageRoi,1-alpha,0,image1);
    image1.copyTo(imageRoi,mask);
    imshow(WINDOW_NAME,image2);
    return 1;
}

void trackbarHandle(int , void*)
{
    double alpha = (double)current_value/100;
    double beta = 1 - alpha;
    cout<<alpha<<endl;
    Mat out_image;
    addWeighted(image1,alpha,imageRoi,beta,0,out_image);
    imshow(WINDOW_NAME,out_image);

    //out_image.copyTo(imageRoi,mask);//目前这个函数使用有问题，因为copyTo函数会一直操作图像
    //imshow(WINDOW_NAME,image1);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "ReadAndShowImage");
    ros::NodeHandle nh;
    namedWindow(WINDOW_NAME,1);
    current_value = 70;
    cout<<"The size of the first image is "<<image1.cols<<" "<<image1.rows<<endl;
    cout<<"The size of the first image is "<<image2.cols<<" "<<image2.rows<<endl;
    //roi_blender();
    //addweight_blender(50);
    createTrackbar("透明度100",WINDOW_NAME,&current_value,MaxValue,trackbarHandle);
    trackbarHandle(current_value,0);
    waitKey(0);
    return 0;
}