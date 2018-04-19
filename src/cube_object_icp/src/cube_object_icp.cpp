#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Image.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class CUBE_OBJECT
{
public:
    CUBE_OBJECT();
    void showCb(PointCloudT::Ptr &input_cloud);
private:
    // 定义和通讯相关的变量
    ros::NodeHandle nh;
    ros::Subscriber sub;

    // 定义和点云数据类型相关的变量
    pcl::PCLPointCloud2* source_cloud2_it;                      // 原始的点云数据
    pcl::PCLPointCloud2::Ptr source_cloud2Ptr;                  // 同上，只是格式不一样
    pcl::PCLPointCloud2::Ptr source_cloud2_down_sampledPtr;     // 降采样之后的原始数据
    PointCloudT::Ptr cloud_model;                               // 理论的物体模型
    PointCloudT::Ptr cloud_down_sampled;                        // 降采样之后的原始数据，但是数据格式变成pcl常用的格式
    PointCloudT::Ptr cloud_pass_through_x;                      // x方向直通滤波结果
    PointCloudT::Ptr cloud_pass_through_y;                      // y方向直通滤波结果
    PointCloudT::Ptr cloud_segmented;                           // 平面分割之后的结果
    PointCloudT::Ptr cloud_outlier_removed;                     // 移除离群点之后的结果，一般也作为最终的点云结果
    Eigen::Matrix4d transformation_matrix;                      // 存储最后的其次变化矩阵
    // 定义函数
    void print4x4Matrix(const Eigen::Matrix4d & matrix);                                                                                    // 输出eigen类型的矩阵
    void down_sample(pcl::PCLPointCloud2::Ptr &source_cloud2Ptr,pcl::PCLPointCloud2::Ptr &source_cloud2_down_sampledPtr,float leafsize);    // 对原始数据进行降采样
    void segmente(PointCloudT::Ptr &input_cloud,PointCloudT::Ptr &cloud_segmented);                                                         // 模型平面分割
    void stastical_removal(PointCloudT::Ptr &input_cloud,PointCloudT::Ptr &cloud_outlier_removed);                                          // 统计滤波去除离群点
    void pose_estimation(PointCloudT::Ptr &cloud_outlier_removed, PointCloudT::Ptr &target_cloud, Eigen::Matrix4d &transformation_matrix);  // 估计位恣
    void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);                                                                      // 点云的回调函数
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cube_object_icp");
    CUBE_OBJECT cube_object_icp;
    ros::spin();
    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    // CUBE_OBJECT cube_object_icp;
    // cube_object_icp.showCb();
    // ros::spin();
    return 0;
}

void CUBE_OBJECT::cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl_conversions::toPCL(*cloud_msg, *source_cloud2_it);                          // 将ros的点云数据格式转换成pcl::PointCloud2,方便进行接下来的降采样
    down_sample(source_cloud2Ptr,source_cloud2_down_sampledPtr,0.002);              // 执行降采样操作，传递的参数是叶子的大小
    pcl::fromPCLPointCloud2(*source_cloud2_down_sampledPtr, *cloud_down_sampled);   // 将pcl::PointCloud2数据格式转换成pcl::PointCloud<POintCloud>
    ROS_INFO("You are in cloud cb");
    // showCb(cloud_down_sampled);
    segmente(cloud_down_sampled,cloud_segmented);                                   // 分割平面
    // showCb(cloud_segmented);
    stastical_removal(cloud_segmented,cloud_outlier_removed);                       // 移除离群点
    // showCb(cloud_outlier_removed);
    pose_estimation(cloud_outlier_removed,cloud_model,transformation_matrix);       // 使用ICP进行位恣估计
    Eigen::Matrix4d trans_inverse;
    trans_inverse = transformation_matrix.inverse();
    print4x4Matrix(trans_inverse);
}

CUBE_OBJECT::CUBE_OBJECT():source_cloud2_it(new pcl::PCLPointCloud2),source_cloud2Ptr(source_cloud2_it),
                           source_cloud2_down_sampledPtr(new pcl::PCLPointCloud2),cloud_pass_through_x(new PointCloudT),
                           cloud_pass_through_y(new PointCloudT),cloud_segmented(new PointCloudT),cloud_outlier_removed (new PointCloudT),
                           cloud_model(new PointCloudT),cloud_down_sampled(new PointCloudT)
{
    sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, &CUBE_OBJECT::cloud_cb,this);
    transformation_matrix = Eigen::Matrix4d::Identity();
    ros::Duration(1.0).sleep();
    if(pcl::io::loadPCDFile("/home/leon/graspdemo/src/cube_object_icp/zhangheng_cube_model.pcd",*cloud_model) == -1)
    {
        ROS_ERROR("Load File Error");
    }
}

// 显示一个固定的点云
void CUBE_OBJECT::showCb(PointCloudT::Ptr &input_cloud)
{
    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (input_cloud);
    //显示直到程序结束，并且不占用主线程
    while (!viewer.wasStopped ()&&ros::ok())
    {
        ROS_INFO("You are in the loop now!");
        ros::Duration(0.1).sleep();
    }
}

void CUBE_OBJECT::print4x4Matrix(const Eigen::Matrix4d & matrix)
{
    printf("Rotation matrix :\n");
    printf("    | %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
    printf("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void CUBE_OBJECT::down_sample(pcl::PCLPointCloud2::Ptr &source_cloud2Ptr,pcl::PCLPointCloud2::Ptr &source_cloud2_down_sampledPtr,float leafsize)
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor_downsample;
    sor_downsample.setInputCloud(source_cloud2Ptr);
    sor_downsample.setLeafSize(leafsize,leafsize,leafsize);
    // sor_downsample.setLeafSize(0.002f, 0.002f, 0.002f);
    sor_downsample.filter(*source_cloud2_down_sampledPtr);
}

void CUBE_OBJECT::segmente(PointCloudT::Ptr &input_cloud,PointCloudT::Ptr &cloud_segmented)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    //Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);
    //Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);  //true corresponds to the target, false corresponds to the plane
    extract.filter(*cloud_segmented);
    ROS_INFO("PointCloud representing the cube segmentation: %d data points. ", cloud_segmented->width * cloud_segmented->height);
}

void CUBE_OBJECT::stastical_removal(PointCloudT::Ptr &input_cloud,PointCloudT::Ptr &cloud_outlier_removed)
{
    pcl::StatisticalOutlierRemoval<PointT> sor_rm_outlier;
    sor_rm_outlier.setInputCloud(input_cloud);
    sor_rm_outlier.setMeanK(50);
    sor_rm_outlier.setStddevMulThresh(2);
    sor_rm_outlier.filter(*cloud_outlier_removed);
}

void CUBE_OBJECT::pose_estimation(PointCloudT::Ptr &cloud_outlier_removed, PointCloudT::Ptr &target_cloud, Eigen::Matrix4d &transformation_matrix)
{
    float sum_x = 0;
    float sum_y = 0;
    float sum_z = 0;
    float trans_x = 0;
    float trans_y = 0;
    float trans_z = 0;

    for(size_t i = 0; i < cloud_outlier_removed->points.size(); ++i)
    {
        sum_x = cloud_outlier_removed->points[i].x + sum_x;
        sum_y = cloud_outlier_removed->points[i].y + sum_y;
        sum_z = cloud_outlier_removed->points[i].z + sum_z;
    }
    trans_x = sum_x / cloud_outlier_removed->points.size();
    trans_y = sum_y / cloud_outlier_removed->points.size();
    trans_z = sum_z / cloud_outlier_removed->points.size();
    // 用三个方向的均值当做平移的初始值

    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (-trans_x, -trans_y, -trans_z);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
        
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaxCorrespondenceDistance(0.01);
    icp.setMaximumIterations(200);
    icp.setInputSource(cloud_outlier_removed);
    icp.setInputTarget(target_cloud);
    icp.align(*cloud_outlier_removed, init_guess);
        
    Eigen::Matrix4d trans_inverse;

    if(icp.hasConverged())
    {
        std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
        std::cout << "\nICP transformation: cloud_model -> cloud_cube" << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
    }
    else    
        ROS_INFO("ICP has not converged!");
    }