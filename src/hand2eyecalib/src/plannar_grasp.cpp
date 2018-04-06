
//这个cpp是为了进行手眼标定系统

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
        // group.setMaxVelocityScalingFactor(0.3);
        // group.setMaxAccelerationScalingFactor(0.2);
        // group.setNamedTarget("look");
        // group.move();
        // ros::Duration(0.5).sleep();

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
        cal_poses[0].position.z = 0.834113;
        cal_poses[0].orientation.x = -0.91615;
        cal_poses[0].orientation.y = 0.399768;
        cal_poses[0].orientation.z = -0.0214981;
        cal_poses[0].orientation.w = 0.0198302;

        //向后
        cal_poses[1].position.x = 0.246834;
        cal_poses[1].position.y = -0.102558;
        cal_poses[1].position.z = 0.594104;
        cal_poses[1].orientation.x = -0.887616;
        cal_poses[1].orientation.y = 0.38696;
        cal_poses[1].orientation.z = -0.193535;
        cal_poses[1].orientation.w = 0.15792;


        //向前
        cal_poses[2].position.x = 0.954965;
        cal_poses[2].position.y = 0.0519097;
        cal_poses[2].position.z = 0.619134;
        cal_poses[2].orientation.x = 0.877096;
        cal_poses[2].orientation.y = -0.40378;
        cal_poses[2].orientation.z = -0.207149;
        cal_poses[2].orientation.w = 0.157331;

        //向左
        cal_poses[3].position.x = 0.271583;
        cal_poses[3].position.y = -0.409755;
        cal_poses[3].position.z = 0.673909;
        cal_poses[3].orientation.x = -0.573487;
        cal_poses[3].orientation.y = 0.67471;
        cal_poses[3].orientation.z = 0.0941883;
        cal_poses[3].orientation.w = 0.454981;

        //向右
        cal_poses[4].position.x = 0.459985;
        cal_poses[4].position.y = 0.569304;
        cal_poses[4].position.z = 0.569947;
        cal_poses[4].orientation.x = 0.86657;
        cal_poses[4].orientation.y = 0.244459;
        cal_poses[4].orientation.z = -0.0269062;
        cal_poses[4].orientation.w = 0.434237;

        cal_poses[5].position.x = 0.504905;
        cal_poses[5].position.y = -0.0195077;
        cal_poses[5].position.z = 0.552653;
        cal_poses[5].orientation.x = -0.923636;
        cal_poses[5].orientation.y = 0.380849;
        cal_poses[5].orientation.z = -0.0402378;
        cal_poses[5].orientation.w = 0.0151954;

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

    visp_hand2eye_calibration::TransformArray object_camera;
    visp_hand2eye_calibration::TransformArray effector_world;

    Eigen::Affine3d matrix_c_e;
    Eigen::Affine3d matrix_o_c;
    Eigen::Affine3d matrix_e_w;
    Eigen::Affine3d matrix_o_w;


    vector<Point2f> image_points_buf;  //缓存每幅图像上检测到的角点
    
    geometry_msgs::Transform hand2eyecalib(visp_hand2eye_calibration::TransformArray& camera_object , visp_hand2eye_calibration::TransformArray& world_effector);
    void getCurrentPoseAndTransform();
    void pose2transform(geometry_msgs::Pose& pose, geometry_msgs::Transform& transform);
    void display(Eigen::Affine3d &matrix);
    void DrawFilledCircle(Mat image,Point2f point);
    void convert_callback(const sensor_msgs::ImageConstPtr& msg);
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "testdemo");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ChessBoard chessboard;
    chessboard.start();
    return 0;
}

void ChessBoard::start()
{
    //移动到初始位置
    Eigen::Matrix3d instrinsic;
    instrinsic<<618.982421875, 0.0, 302.331787109375,0.0, 624.7108154296875, 233.72047424316406,0, 0 ,1;
    move.group.setPoseTarget(cal_poses[0]);
    move.group.setMaxAccelerationScalingFactor(0.2);
    move.group.setMaxVelocityScalingFactor(0.3);
    move.group.move();
    process(image_in);
    ros::Duration(1).sleep();

    //计算标定板坐标系到基座坐标系的总变换
    tf2::convert(pose_o_c,matrix_o_c);
    getCurrentPoseAndTransform();
    tf2::convert(pose_e_w,matrix_e_w);
    matrix_o_w = matrix_e_w * matrix_c_e * matrix_o_c;
    ros::Duration(1).sleep();
    ROS_INFO("The transform between object and world is ");
    display(matrix_o_w);
    tf2::convert(matrix_o_w,pose_e_w);

    //记录初始位置的旋转矩阵和平移矩阵
    Eigen::Matrix3d rotation;
    rotation = matrix_o_c.rotation();
    Eigen::Vector3d translation;
    translation = matrix_o_c.translation();
    ROS_INFO_STREAM("Rotation Is\n"<<rotation);
    ROS_INFO_STREAM("Tranlation Is\n"<<translation);
    ros::Duration(0.1).sleep();

    std::vector<Eigen::Vector3d> points;
    
    for(int row = 0 ; row < 6 ; row++)
    {
        for(int col = 0 ; col < 9 ; col ++)
        {
            move.group.setPoseTarget(cal_poses[0]);
            move.group.setMaxAccelerationScalingFactor(0.2);
            move.group.setMaxVelocityScalingFactor(0.3);
            move.group.move();
            image_points_buf.clear();
            process(image_in);
            ros::Duration(0.1).sleep();

            Point2d corner_point = image_points_buf[row*9+col];//第ij个点的像素坐标，需要转换到图像坐标
            Eigen::Vector3d image_corner_3d;
            image_corner_3d[0] = corner_point.x;
            image_corner_3d[1] = corner_point.y;
            image_corner_3d[2] = 1;

            image_corner_3d = instrinsic.inverse()*image_corner_3d;
            Eigen::Vector3d Ow = - rotation.transpose() * translation;
            Eigen::Vector3d Iw = rotation.transpose() * (image_corner_3d - translation);
            Eigen::Vector3d Dw = Iw - Ow;
            Eigen::Vector3d Pw;
            Pw[0] = Ow[0] - Ow[2]*Dw[0]/Dw[2];
            Pw[1] = Ow[1] - Ow[2]*Dw[1]/Dw[2];
            Pw[2] = 0;
            //ROS_INFO_STREAM("The Translation of the 3 line and 4 row corner to the frame is \n"<<Pw);
            points.push_back(Pw);


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
            pose_e_w.position.z = pose_e_w.position.z + 0.2;

            move.group.setPoseTarget(pose_e_w);
            move.group.setMaxAccelerationScalingFactor(0.2);
            move.group.setMaxVelocityScalingFactor(0.3);
            move.group.move();
            ros::Duration(0.1).sleep();
        }
        ROS_INFO_STREAM("You are here");
    }

    for(int i = 0 ; i < 6 ; i ++)
    {
        using namespace std;
        for(int j = 0 ; j < 9 ; j ++)
        {
            cout<<setw(20)<<points[i*9+j][0]<<" "<<setw(20)<<points[i*9+j][1]<<std::endl;
        }
        std::cout<<std::endl;
    }
}

geometry_msgs::Transform ChessBoard::hand2eyecalib(visp_hand2eye_calibration::TransformArray& camera_object , visp_hand2eye_calibration::TransformArray& world_effector)
{
    std::vector<vpHomogeneousMatrix> cMo_vec;
    std::vector<vpHomogeneousMatrix> wMe_vec;
    for(unsigned int i=0;i<camera_object.transforms.size();i++)
    {
      cMo_vec.push_back(visp_bridge::toVispHomogeneousMatrix(camera_object.transforms[i]));
      wMe_vec.push_back(visp_bridge::toVispHomogeneousMatrix(world_effector.transforms[i]));
    }
    if (camera_object.transforms.size() != world_effector.transforms.size() || world_effector.transforms.size() < 2)
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
    geometry_msgs::Transform transform_c_e;
    transform_c_e = visp_bridge::toGeometryMsgsTransform(eMc);
    return transform_c_e;
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
        //得到转换后的消息类型 
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
    //vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */  

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
    //for(auto &a:world_points)cout<<a<<" ";

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
