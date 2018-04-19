
//让机械臂移动到6个位置，然后计算相机到末端的位恣
//系统的输出是针对眼在手上的系统，输出相机到机械臂末端的位姿变换矩阵

#include<ros/ros.h>
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
        group.setMaxVelocityScalingFactor(0.3);
        group.setMaxAccelerationScalingFactor(0.2);
        group.setNamedTarget("look");
        //group.move();
        ros::Duration(0.5).sleep();

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
        cal_poses.resize(6);

        //正常位恣
        cal_poses[0].position.x = 0.530;
        cal_poses[0].position.y = -0.063;
        cal_poses[0].position.z = 0.544;
        cal_poses[0].orientation.x = -0.695;
        cal_poses[0].orientation.y = 0.714;
        cal_poses[0].orientation.z = -0.007;
        cal_poses[0].orientation.w = 0.084;

        //向后
        cal_poses[1].position.x = 0.156;
        cal_poses[1].position.y = -0.131;
        cal_poses[1].position.z = 0.409;
        cal_poses[1].orientation.x = 0.648;
        cal_poses[1].orientation.y = -0.646;
        cal_poses[1].orientation.z = 0.193;
        cal_poses[1].orientation.w = -0.353;


        //向前
        cal_poses[2].position.x = 0.926;
        cal_poses[2].position.y = -0.023;
        cal_poses[2].position.z = 0.567;
        cal_poses[2].orientation.x = 0.759;
        cal_poses[2].orientation.y = -0.616;
        cal_poses[2].orientation.z = -0.169;
        cal_poses[2].orientation.w = 0.122;

        //向左
        cal_poses[3].position.x = 0.488;
        cal_poses[3].position.y = -0.473;
        cal_poses[3].position.z = 0.523;
        cal_poses[3].orientation.x = -0.599;
        cal_poses[3].orientation.y = 0.692;
        cal_poses[3].orientation.z = 0.201;
        cal_poses[3].orientation.w = 0.349;

        //向右
        cal_poses[4].position.x = 0.556;
        cal_poses[4].position.y = 0.319;
        cal_poses[4].position.z = 0.488;
        cal_poses[4].orientation.x = 0.731;
        cal_poses[4].orientation.y = -0.642;
        cal_poses[4].orientation.z = 0.2;
        cal_poses[4].orientation.w = 0.113;

        // 中间
        cal_poses[5].position.x = 0.444;
        cal_poses[5].position.y = -0.152;
        cal_poses[5].position.z = 0.805;
        cal_poses[5].orientation.x = -0.17;
        cal_poses[5].orientation.y = 0.968;
        cal_poses[5].orientation.z = 0.036;
        cal_poses[5].orientation.w = 0.183;

        cv::namedWindow(INPUT);  
        cv::namedWindow(OUTPUT);  
        out_msg_flag = 0;
    }
    void start();
    void process(cv::Mat& image);
private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    cv::Mat image_out;
    cv::Mat image_in;
    move_robot move;
    std::vector<geometry_msgs::Pose> cal_poses;
    int out_msg_flag;

    geometry_msgs::Pose pose_o_c;
    geometry_msgs::Pose pose_e_w;
    geometry_msgs::Transform transform_o_c;
    geometry_msgs::Transform transform_e_w;
    geometry_msgs::Transform transform_c_e;
    visp_hand2eye_calibration::TransformArray object_camera;
    visp_hand2eye_calibration::TransformArray effector_world;

    geometry_msgs::Transform hand2eyecalib(visp_hand2eye_calibration::TransformArray&, visp_hand2eye_calibration::TransformArray&);
    void getCurrentPoseAndTransform();
    void pose2transform(geometry_msgs::Pose& pose, geometry_msgs::Transform& transform);
    void display(Eigen::Affine3d &matrix);
    void DrawFilledCircle(Mat image,Point2f point);
    void convert_callback(const sensor_msgs::ImageConstPtr& msg);
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hand2eyecalibration");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ChessBoard chessboard;
    chessboard.start();
    return 0;
}

void ChessBoard::start()
{
    //总的矩阵还没存储
    for(int i = 0 ; i < 6 ;i ++)
    {
        move.group.setPoseTarget(cal_poses[i]);
        move.group.setMaxAccelerationScalingFactor(0.2);
        move.group.setMaxVelocityScalingFactor(0.3);
        move.group.move();
        process(image_in);
        //标定板到相机
        ros::Duration(1).sleep();
        pose2transform(pose_o_c,transform_o_c);
        getCurrentPoseAndTransform();
        //末端到基座的
        pose2transform(pose_e_w,transform_e_w);
        object_camera.transforms.push_back(transform_o_c);
        effector_world.transforms.push_back(transform_e_w);
    }
    transform_c_e = hand2eyecalib(object_camera , effector_world);
    ROS_INFO_STREAM("The Tranform between camera and effector is "<<transform_c_e);
}
geometry_msgs::Transform ChessBoard::hand2eyecalib(visp_hand2eye_calibration::TransformArray& object_camera , visp_hand2eye_calibration::TransformArray& effector_world)
{
    std::vector<vpHomogeneousMatrix> cMo_vec;
    std::vector<vpHomogeneousMatrix> wMe_vec;
    for(unsigned int i=0;i<object_camera.transforms.size();i++)
    {
      cMo_vec.push_back(visp_bridge::toVispHomogeneousMatrix(object_camera.transforms[i]));
      wMe_vec.push_back(visp_bridge::toVispHomogeneousMatrix(effector_world.transforms[i]));
    }
    if (object_camera.transforms.size() != effector_world.transforms.size() || effector_world.transforms.size() < 2)
    {
      ROS_ERROR("transformation vectors have different sizes or contain too few elements");
      exit(1);
    }
    ROS_INFO("computing...");
    vpHomogeneousMatrix eMc;
    #if VISP_VERSION_INT > (2<<16 | 6<<8 | 1)
    vpCalibration::calibrationTsai(cMo_vec, wMe_vec, eMc);
    #else
    vpCalibration::calibrationTsai(cMo_vec.size(),&(cMo_vec[0]),&(wMe_vec[0]),eMc);
    #endif
    geometry_msgs::Transform transform_e_c;
    transform_e_c = visp_bridge::toGeometryMsgsTransform(eMc);
    return transform_e_c;
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
}
void ChessBoard::process(cv::Mat& image)
{
    using namespace cv;
    using namespace std;
    cv::cvtColor(image,image_out,CV_RGB2GRAY);
    //此时image_out已经是灰度图像了
    cv::imshow(OUTPUT,image_out);
    cv::waitKey(1);
    //下面需要得到标定板相对于相机的坐标变换
    Size image_size;  /* 图像的尺寸 */  
    Size board_size = Size(9,6);//标定板的格数
    vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */  

    image_size.width = image.cols;
    image_size.height = image.rows;
        
    if(out_msg_flag == 0)
    {
        out_msg_flag = 1;
        ROS_INFO_STREAM("The width of the image is "<<image_size.width);
        ROS_INFO_STREAM("The height of the image is "<<image_size.height);
    }
    if (0 == findChessboardCorners(image,board_size,image_points_buf))  
    {     
        ROS_WARN("can not find chessboard corners!");  
        //exit(1);         
    } 
    else   
    {  
        Mat view_gray;  
        cvtColor(image,view_gray,CV_RGB2GRAY);  
        find4QuadCornerSubpix(view_gray,image_points_buf,Size(5,5)); //对粗提取的角点进行精确化  
        drawChessboardCorners(view_gray,board_size,image_points_buf,false); //用于在图片中标记角点
        DrawFilledCircle(view_gray,image_points_buf[0]);
        DrawFilledCircle(view_gray,image_points_buf[1]);
        //DrawFilledCircle(view_gray,image_points_buf[53]);
        imshow("Camera Calibration",view_gray);//显示图片  
        waitKey(10);  
    } 
    if(0)
    {
        ROS_INFO_STREAM("The size of the corner points are "<<image_points_buf.size());
        for(int i = 0 ; i < 54; i ++)
        {
            if(i%9==0&&i!=0)
                cout<<endl;
            cout<<image_points_buf[i]<<" ";
        }
        cout<<endl;
    }
    std::vector<Point3f> world_points;
    int rows = 6 ; 
    int cols = 9 ;
    Point3f temp_world_point;
    double chessboard_distance = 0.0333;//棋盘格每个格子的大小，单位是米
    for(int row = 0 ; row < rows ; row ++)
    {
        for(int col = 0 ; col < cols ; col ++)
        {
            temp_world_point.z = 0 ; 
            temp_world_point.x = chessboard_distance*col;//水平是x
            temp_world_point.y = chessboard_distance*row;//竖直是y
            world_points.push_back(temp_world_point);
        }
    }

    Mat cameraMatrix(3,3,CV_32F);//相机内参数
    vector<float> distCoeff(0);
    float tempMatrix[3][3] = { { 618.982421875, 0.0, 302.331787109375 }, { 0.0, 624.7108154296875, 233.72047424316406 }, { 0, 0 ,1} };
    for (int i = 0; i < 3;i++)
    {
        for (int j = 0; j < 3;j++)
        {
            cameraMatrix.at<float>(i, j) = tempMatrix[i][j];
        }
    }
    Mat rvec, tvec;
    solvePnP(world_points, image_points_buf, cameraMatrix, distCoeff, rvec, tvec);
    Rodrigues(rvec, rvec);

    Eigen::Affine3d matrix_o_c;
    Eigen::Affine3d matrix_c_o;
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
}
