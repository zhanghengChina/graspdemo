#include "ros/ros.h"
#include<iostream>    
#include<cv_bridge/cv_bridge.h> 
#include<sensor_msgs/image_encodings.h> 
#include<image_transport/image_transport.h>
#include "opencv2/opencv.hpp"

const std::string win_name = "原始图";


class CONTOUR
{
public:
    CONTOUR();
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_; 
    image_transport::Subscriber image_sub_;  
    image_transport::Publisher image_pub_; 

    // 定义函数
    void convert_callback(const sensor_msgs::ImageConstPtr& msg);               // 图像的回调函数
    void image_process(cv::Mat img);                                            // 图像处理相关的函数
};


int main(int argc, char *argv[])
{
    ros::init(argc,argv,"find_contour");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    CONTOUR find_contour;
    ros::spin();
    return 0;
}


CONTOUR::CONTOUR():it_(nh_)
{
    image_sub_ = it_.subscribe("camera/rgb/image_rect_color", 1, &CONTOUR::convert_callback, this);  
    image_pub_ = it_.advertise("/realsense/image", 1); //定义图象发布器   
}

void CONTOUR::convert_callback(const sensor_msgs::ImageConstPtr& msg)   
{  
    cv_bridge::CvImagePtr cv_ptr; // 声明一个CvImage指针的实例  
    try  
    {  
        cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8); //将ROS消息中的图象信息提取，生成新cv类型的图象，复制给CvImage指针  
    }  
    catch(cv_bridge::Exception& e)  //异常处理  
    {  
        ROS_ERROR("cv_bridge exception: %s", e.what());  
        return;  
    }  
    image_process(cv_ptr->image); //得到了cv::Mat类型的图象，在CvImage指针的image中，将结果传送给处理函数
    image_pub_.publish(cv_ptr->toImageMsg());     
}  

void CONTOUR::image_process(cv::Mat img)   
{  
    using namespace cv;
    // std::cout<<img.size()<<std::endl；
    Mat img1(img,Rect(80,80,400,300));
    ROS_INFO_STREAM("Image size"<<img.size());
    Mat gray_image,canny_output;
    std::vector<std::vector<Point> > vContours;
    std::vector<Vec4i> vHierarchy;
    
    
    ROS_INFO("You are in processing function!");
    cvtColor(img1, gray_image, CV_RGB2GRAY);  //转换成灰度图象
    blur(gray_image,gray_image,Size(3,3));

    Canny(gray_image,canny_output,60,160,3);

    findContours(canny_output,vContours,vHierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE,Point(0,0));
    // 画出轮廓
    Mat drawing = Mat::zeros(canny_output.size(),CV_8UC3);
    for(int i = 0; i<vContours.size(); i++)
    {
        Scalar color = Scalar(100,100,100);
        drawContours(drawing,vContours,i,color,2,8,vHierarchy,0,Point());
    }
    imshow("原始图像",drawing);
    waitKey(5);
}