
//这个节点通过ROS读取图像 传给opencv，然后进行图像处理

#include<ros/ros.h> 
#include<iostream>    
#include<cv_bridge/cv_bridge.h> 
#include<sensor_msgs/image_encodings.h> 
#include<image_transport/image_transport.h>
#include "opencv2/opencv.hpp"

static const std::string INPUT = "Input"; 
static const std::string OUTPUT = "Output"; 

class RGB_GRAY  
{  
private:  
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_; 
    image_transport::Subscriber image_sub_;  
    image_transport::Publisher image_pub_; 
public:  
    RGB_GRAY()  
      :it_(nh_)  
    {  
        image_sub_ = it_.subscribe("camera/rgb/image_rect_color", 1, &RGB_GRAY::convert_callback, this);  
        image_pub_ = it_.advertise("/realsense/image", 1); //定义图象发布器  
        cv::namedWindow(INPUT,CV_WINDOW_NORMAL);  
        cv::namedWindow(OUTPUT,CV_WINDOW_NORMAL);  
    }  
    ~RGB_GRAY()   
    {  
         cv::destroyWindow(INPUT);  
         cv::destroyWindow(OUTPUT);  
    }  

    void convert_callback(const sensor_msgs::ImageConstPtr& msg)   
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

    //图像处理函数
    void image_process(cv::Mat img)   
    {  
       using namespace cv;
       imshow(INPUT,img);
    //    std::vector<int> vect;
    //    vect.push_back(CV_IMWRITE_PNG_COMPRESSION);
    //    imwrite("image.png",img,vect);
    //    ROS_INFO("Writing Done");
       waitKey(5);
/*
       Mat element = getStructuringElement(MORPH_RECT,Size(15,15));
       Mat dstImage;
       erode(img,dstImage,element);
       imshow(OUTPUT,dstImage);
       waitKey(5);//单位是毫秒      
       Mat img_out;  
       cvtColor(img, img_out, CV_RGB2GRAY);  //转换成灰度图象  
       imshow(INPUT, img);  
       imshow(OUTPUT, img_out);  
       waitKey(5); 
*/ 
    }
};


int main(int argc, char** argv)  
{  
    ros::init(argc, argv, "RGB");  
    RGB_GRAY obj;  
    ros::spin();  
}