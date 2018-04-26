#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include "ros/ros.h"
using namespace cv;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dft");
    ros::NodeHandle nh;

    Mat srcImage = imread("/home/leon/graspdemo/src/opencvtest/src/dft.jpg", 0);
	if(!srcImage.data ) { printf("打开文件错误，请指定正确的文件路径 \n"); return false; } 
	imshow("原始图像" , srcImage);   
    std::cout<<"原始图像的大小是"<<std::endl;
    std::cout<<srcImage.size().height<<" "<<srcImage.size().width<<std::endl;

	int m = getOptimalDFTSize( srcImage.rows );
	int n = getOptimalDFTSize( srcImage.cols ); 
    // 当尺寸是2,3,5的倍数的时候，处理的速度会比较快

    std::cout<<"扩展之后的行列数是："<<std::endl;
    std::cout<<"m = "<<m<<" "<<"n = "<<n<<std::endl;
	Mat padded;  
	copyMakeBorder(srcImage, padded, 0, m - srcImage.rows, 0, n - srcImage.cols, BORDER_CONSTANT, Scalar::all(0));
    imshow("扩展之后的图像",padded); //会看到图像底部有明显的黑边

	Mat planes[] = {Mat_<float>(padded), Mat::zeros(padded.size(), CV_32F)};
	Mat complexI;
	merge(planes, 2, complexI);         

	dft(complexI, complexI);    // 就地的进行傅里叶变换       

	split(complexI, planes); 
	magnitude(planes[0], planes[1], planes[0]);// planes[0] = magnitude  
	Mat magnitudeImage = planes[0];

	magnitudeImage += Scalar::all(1);
	log(magnitudeImage, magnitudeImage);//执行对数操作，因为正常情况下，F（0,0）处的值代表的是图像的平均灰度，数值比较大，所以为了显示方便，我们需要执行一个对数操作
	magnitudeImage = magnitudeImage(Rect(0, 0, magnitudeImage.cols & -2, magnitudeImage.rows & -2));
    // normalize(magnitudeImage, magnitudeImage, 0, 1, CV_MINMAX); 
    // imshow("原始的dft图像",magnitudeImage);
    // 因为dft是周期延拓的，所以在原始的magnitudeImage的四个角落里面代表的是F(0,0)，为了显示的方便，需要将F(0,0)移到图像的中间
	int cx = magnitudeImage.cols/2;
	int cy = magnitudeImage.rows/2;
	Mat q0(magnitudeImage, Rect(0, 0, cx, cy));   // 
	Mat q1(magnitudeImage, Rect(cx, 0, cx, cy));  // 
	Mat q2(magnitudeImage, Rect(0, cy, cx, cy));  // 
	Mat q3(magnitudeImage, Rect(cx, cy, cx, cy)); // 
	Mat tmp;                           
	q0.copyTo(tmp);
	q3.copyTo(q0);
	tmp.copyTo(q3);
	q1.copyTo(tmp);                 
	q2.copyTo(q1);
	tmp.copyTo(q2);
    
	normalize(magnitudeImage, magnitudeImage, 0, 1, CV_MINMAX); 
	// 虽然上面执行的对数操作，但是还是显示不方便，为了显示的方便，我们最好将原始的数据归一化到0-1之间

	imshow("DFT", magnitudeImage);    
	waitKey();
    return 0;
}