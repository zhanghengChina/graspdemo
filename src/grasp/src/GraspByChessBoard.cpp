
//本程序的作用是通过在木块上面贴上标定板来进行简单的目标抓取
//核心思想是检测角点，提取对角线上的两个角点，计算这两个角点对应的真是世界坐标，然后求取对应的世界坐标的中心点即为物体的真实中心点
//因为从从标定板到相机的坐标变换已经包含了旋转信息，所以中心点到标定板的原点之间不需要进行额外的变换

#include<ros/ros.h>
#include "iomanip"
#include<iostream>   
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>  
#include<image_transport/image_transport.h>
#include "moveit/move_group_interface/move_group.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "control_msgs/FollowJointTrajectoryActionGoal.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/String.h"
#include "opencv2/opencv.hpp"
#include <tf2/convert.h>
#include "visp_hand2eye_calibration/TransformArray.h"
#include <visp_bridge/3dpose.h>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <tf2_eigen/tf2_eigen.h>
#include <visp/vpCalibration.h>
#include <visp/vpHomogeneousMatrix.h>


static const std::string INPUT = "Input"; //定义输入窗口名称  
static const std::string OUTPUT = "Output"; //定义输出窗口名称  
#define WINDOW_WIDTH  640
//下面的信息是关于棋盘格的信息
const int chess_row = 6;
const int chess_col = 9;
const double chessboard_distance = 0.0333;//棋盘格每个格子的大小，单位是米

// const int chess_row = 6;
// const int chess_col = 7;
// const double chessboard_distance = 0.0122;//棋盘格每个格子的大小，单位是米

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> Client;
using namespace cv;

class move_robot
{//定义一个和运动相关的类，主要用来获取当前的位恣
public:
    move_robot():group("manipulator"),robot_model_loader("robot_description"),kinematic_model(robot_model_loader.getModel()),
                kinematic_state(*group.getCurrentState()),joint_model_group(kinematic_model->getJointModelGroup("manipulator"))
    {
        Client ac("follow_joint_trajectory", true);
        if (!ac.waitForServer(ros::Duration(2.0)))
        {
            ROS_ERROR("Could not connect to action server");
            exit(-1);
        }
        else
        {
            ROS_INFO("Connecte with UR10 Now!");
        }
    }
    moveit::planning_interface::MoveGroup group;
    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    moveit::core::RobotState kinematic_state;
    const robot_state::JointModelGroup* joint_model_group;
};

class ChessBoard
{
public:
    ChessBoard():it_(nh)
    {
        image_sub_ = it_.subscribe("camera/rgb/image_rect_color", 1, &ChessBoard::convert_callback, this);
        image_pub_ = it_.advertise("/realsense/processed/image",1);
        cal_poses.resize(6);

        //之前得到的相机相对末端的变换
        pose_c_e.position.x = 0.0956598;
        pose_c_e.position.y = -0.0472065;
        pose_c_e.position.z = 0.0130916;
        pose_c_e.orientation.x = 0.00487664;
        pose_c_e.orientation.y = 0.0134572;
        pose_c_e.orientation.z = 0.385364;
        pose_c_e.orientation.w = 0.922654;
        tf2::convert(pose_c_e,matrix_c_e);  
        //正常位恣
        cal_poses[0].position.x = 0.554535;
        cal_poses[0].position.y = -0.0190335;
        cal_poses[0].position.z = 0.334113;
        cal_poses[0].orientation.x = -0.91615;
        cal_poses[0].orientation.y = 0.399768;
        cal_poses[0].orientation.z = -0.0214981;
        cal_poses[0].orientation.w = 0.0198302;
        cv::namedWindow(INPUT);  
        cv::namedWindow(OUTPUT);  
        out_msg_flag = 0;
    }
    void start();
    int process(cv::Mat& image);
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat image_out;
    cv::Mat image_in;
    move_robot move;
    std::vector<geometry_msgs::Pose> cal_poses;
    int out_msg_flag;

    geometry_msgs::Pose pose_o_c;
    geometry_msgs::Pose pose_e_w;
    geometry_msgs::Pose pose_c_e;

    geometry_msgs::Transform transform_o_c;
    geometry_msgs::Transform transform_e_w;
    geometry_msgs::Transform transform_c_e;

    Eigen::Affine3d matrix_c_e;
    Eigen::Affine3d matrix_o_c;
    Eigen::Affine3d matrix_e_w;
    Eigen::Affine3d matrix_o_w;


    vector<Point2f> image_points_buf;  //缓存每幅图像上检测到的角点
    void getCurrentPoseAndTransform();
    void pose2transform(geometry_msgs::Pose& pose, geometry_msgs::Transform& transform);
    void display(Eigen::Affine3d &matrix);
    void DrawFilledCircle(Mat image,Point2f point);
    void convert_callback(const sensor_msgs::ImageConstPtr& msg);
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "GraspByChessBoard");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ChessBoard chessboard;
    chessboard.start();
    return 0;
}

void ChessBoard::start()
{
    int image_process_glag;
    //移动到初始位置
    Eigen::Matrix3d instrinsic;
    instrinsic<<618.982421875, 0.0, 302.331787109375,0.0, 624.7108154296875, 233.72047424316406,0, 0 ,1;
    move.group.setPoseTarget(cal_poses[0]);//到达一个可以看见标定板的位置
    move.group.setMaxAccelerationScalingFactor(0.2);
    move.group.setMaxVelocityScalingFactor(0.3);
    move.group.move();

    while (process(image_in) == 0)
    {   
        //防止因为检测不到棋盘格的角点而引起的程序异常退出
        ROS_ERROR("Can not detect the chessboard,please move it, thanks");
        ros::Duration(3).sleep();
    }
    //计算标定板坐标系到基座坐标系的总变换
    getCurrentPoseAndTransform();//返回pose_e_w
    tf2::convert(pose_e_w,matrix_e_w);
    matrix_o_w = matrix_e_w * matrix_c_e * matrix_o_c;
    // ROS_INFO("The transform between object and world is ");
    // display(matrix_o_w);

    //下面需要记录考虑物体朝时候，棋盘格中心点到棋盘格远点的坐标变换


    //记录初始位置的旋转矩阵和平移矩阵
    Eigen::Matrix3d rotation;
    rotation = matrix_o_c.rotation();
    Eigen::Vector3d translation;
    translation = matrix_o_c.translation();

    Point2f left_up_point, right_up_point, left_down_point, right_down_point;
    left_up_point = image_points_buf[0];
    right_up_point = image_points_buf[chess_col- 1];
    left_down_point = image_points_buf[(chess_row-1)*chess_col];
    right_down_point = image_points_buf[chess_col*chess_row -1];

    Eigen::Vector3d left_up_point_eigen, right_down_point_eigen;
    left_up_point_eigen[0] = left_up_point.x;
    left_up_point_eigen[1] = left_up_point.y;
    left_up_point_eigen[2] = 1;
    right_down_point_eigen[0] = right_down_point.x;
    right_down_point_eigen[1] = right_down_point.y;
    right_down_point_eigen[2] = 1;
    
    Eigen::Vector3d Ow_right_down_point, Iw_right_down_point, Dw_right_down_point, Pw_right_down_point;
    Eigen::Vector3d Ow_left_up_point, Iw_left_up_point, Dw_left_up_point, Pw_left_up_point;

    right_down_point_eigen = instrinsic.inverse()*right_down_point_eigen;
    left_up_point_eigen = instrinsic.inverse()*left_up_point_eigen;
    Ow_right_down_point = - rotation.transpose() * translation;
    Ow_left_up_point = - rotation.transpose() * translation;
    Iw_right_down_point = rotation.transpose() * (right_down_point_eigen - translation);
    Iw_left_up_point = rotation.transpose() * (left_up_point_eigen - translation);
    Dw_right_down_point = Iw_right_down_point - Ow_right_down_point;
    Dw_left_up_point = Iw_left_up_point - Ow_left_up_point;
    Pw_right_down_point[0] = Ow_right_down_point[0] - Ow_right_down_point[2]*Dw_right_down_point[0]/Dw_right_down_point[2];
    Pw_right_down_point[1] = Ow_right_down_point[1] - Ow_right_down_point[2]*Dw_right_down_point[1]/Dw_right_down_point[2];
    Pw_right_down_point[2] = 0;
    ROS_INFO_STREAM("left_up_point is\n"<<Pw_left_up_point);
    ROS_INFO_STREAM("right_down_point is\n"<<Pw_right_down_point);
    Pw_left_up_point[0] = Ow_left_up_point[0] - Ow_left_up_point[2]*Dw_left_up_point[0]/Dw_left_up_point[2];
    Pw_left_up_point[1] = Ow_left_up_point[1] - Ow_left_up_point[2]*Dw_left_up_point[1]/Dw_left_up_point[2];
    Pw_left_up_point[2] = 0;
    Eigen::Vector3d Pw;
    Pw[0] = (Pw_left_up_point[0] + Pw_right_down_point[0])/2;
    Pw[1] = (Pw_left_up_point[1] + Pw_right_down_point[1])/2;
    Pw[2] = 0;
    ROS_INFO_STREAM("Middle Point is \n"<<Pw);

    Eigen::Affine3d matrix_corner_o;
    geometry_msgs::Pose corner_pose;
    corner_pose.position.x = Pw[0];
    corner_pose.position.y = Pw[1];
    corner_pose.position.z = Pw[2];
    corner_pose.orientation.x = corner_pose.orientation.y = corner_pose.orientation.z = 0;
    corner_pose.orientation.w = 1;
    tf2::convert(corner_pose,matrix_corner_o);
    Eigen::Affine3d total_matrix ;
    total_matrix = matrix_o_w * matrix_corner_o;

    tf2::convert(total_matrix,pose_e_w);
    pose_e_w.position.z = pose_e_w.position.z + 0.05;

    move.group.setPoseTarget(pose_e_w);
    move.group.setMaxAccelerationScalingFactor(0.2);
    move.group.setMaxVelocityScalingFactor(0.3);
    move.group.move();
    ros::Duration(0.1).sleep();
}
void ChessBoard::DrawFilledCircle(Mat image,Point2f point)
{
    int thickness = -1;
    int linetype = 8;
    circle(image,
        point,
        WINDOW_WIDTH/100,
        Scalar(0,0,255),
        thickness,
        linetype);
}
void ChessBoard::pose2transform(geometry_msgs::Pose& pose, geometry_msgs::Transform& transform)
{
    transform.translation.x = pose.position.x;
    transform.translation.y = pose.position.y;
    transform.translation.z = pose.position.z;
    transform.rotation = pose.orientation;
}
void ChessBoard::display(Eigen::Affine3d &matrix)
{
    std::cout<<"The Transpose Is"<<std::endl;
    for(int i = 0 ; i < 4; i ++)
    {
        for(int j = 0 ; j < 4 ; j ++)
        {
            std::cout<<std::setw(15)<<matrix(i,j);
        }
        std::cout<<std::endl;
    }
}
void ChessBoard::convert_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try  
    {  
        cv_ptr =  cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    }  
    catch(cv_bridge::Exception& e)
    {  
        ROS_ERROR("cv_bridge exception: %s", e.what());  
        return;  
    }  
    image_in = cv_ptr->image;
}
void ChessBoard::getCurrentPoseAndTransform()
{
    
    pose_e_w = move.group.getCurrentPose(move.group.getEndEffectorLink()).pose;
    ROS_INFO_STREAM("Current Pose is "<<pose_e_w);
    tf2::convert(pose_e_w,matrix_e_w);
}
int ChessBoard::process(cv::Mat& image)
{
    using namespace cv;
    using namespace std;
    //cv::cvtColor(image,image_out,CV_RGB2GRAY);
    cv::imshow(OUTPUT,image);
    cv::waitKey(1);
    Size image_size;  /* 图像的尺寸 */  
    Size board_size = Size(chess_col,chess_row);//标定板的格数
    //vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */  

    image_size.width = image.cols;
    image_size.height = image.rows;
        
    if(out_msg_flag == 0)
    {
        //输出图像数据的大小，并且out_msg_flag控制下使得该输出只输出一次
        out_msg_flag = 1;
        ROS_INFO_STREAM("The width of the image is "<<image_size.width);
        ROS_INFO_STREAM("The height of the image is "<<image_size.height);
    }
    if (0 == findChessboardCorners(image,board_size,image_points_buf))  
    {     
        ROS_WARN("can not find chessboard corners!");   
        return 0;      
    } 
    else   
    {  
        Mat view_gray;  
        cvtColor(image,view_gray,CV_RGB2GRAY);  
        find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); //对粗提取的角点进行精确化  
        drawChessboardCorners(view_gray,board_size,image_points_buf,false); //用于在图片中标记角点
        DrawFilledCircle(view_gray,image_points_buf[0]);
        DrawFilledCircle(view_gray,image_points_buf[1]);
        //用来测试角点从左上角还是右下角开始的
        imshow("Camera Calibration",view_gray);//显示图片  
        waitKey(1);  
    } 
    if(0)
    {
        ROS_INFO_STREAM("The size of the corner points are "<<image_points_buf.size());
        for(int i = 0 ; i < chess_col*chess_row; i ++)
        {
            if(i%chess_col==0&&i!=0)
                cout<<endl;
            cout<<image_points_buf[i]<<" ";
        }
        cout<<endl;
    }
    std::vector<Point3f> world_points;
    Point3f temp_world_point;
    for(int row = 0 ; row < chess_row ; row ++)
    {
        for(int col = 0 ; col < chess_col ; col ++)
        {
            temp_world_point.z = 0 ; 
            temp_world_point.x = chessboard_distance*col;//水平是x
            temp_world_point.y = chessboard_distance*row;//竖直是y
            world_points.push_back(temp_world_point);
        }
    }

    Mat cameraMatrix(3,3,CV_32F);//相机内参数,32位的float类型的数据
    vector<float> distCoeff(0);//因为直接使用的畸变矫正之后的图像，因此不用再考虑畸变的问题
    float tempMatrix[3][3] = { { 618.982421875, 0.0, 302.331787109375 }, { 0.0, 624.7108154296875, 233.72047424316406 }, { 0, 0 ,1} };
    for (int i = 0; i < 3;i++)
    {
        for (int j = 0; j < 3;j++)
        {
            cameraMatrix.at<float>(i, j) = tempMatrix[i][j];
        }
    }
    Mat rvec, tvec;
    solvePnP(world_points, image_points_buf, cameraMatrix, distCoeff, rvec, tvec);//利用对极几何来求解标定板到相机的坐标变换
    Rodrigues(rvec, rvec);
    
    for(int i = 0 ; i < 3 ; i ++)
    {
        for(int j = 0 ; j < 3 ; j ++)
        {
            matrix_o_c(i,j) = rvec.at<double>(i,j);
        }
        matrix_o_c(i,3) = tvec.at<double>(i);
    }
    matrix_o_c(3,0) = matrix_o_c(3,1) = matrix_o_c(3,2) = 0;
    matrix_o_c(3,3) = 1;
    display(matrix_o_c);
    tf2::convert(matrix_o_c,pose_o_c);
    cout<<endl<<endl<<endl;
    return 1;
}



