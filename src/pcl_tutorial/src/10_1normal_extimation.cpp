// 该节点的内容是计算一个点云的法线，其中在可视化节点中已经实现了部分内容。

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include "ros/ros.h"

int main (int argc, char **argv)
{
    ros::init(argc,argv,"normal_estimation");
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("/home/leon/graspdemo/src/pcl_tutorial/table_scene_lms400.pcd", *cloud);

    ros::Time start_time = ros::Time::now();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);
    // 设置点云输入
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);
    // 这一块的对象都需要设置搜索方法
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    // 点云法线的存储
    ne.setRadiusSearch (0.03);
    // 使用半径搜索的方法
    ne.compute (*cloud_normals);
    // 重载的compute函数

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.0);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, cloud_normals);
    ROS_INFO_STREAM("Costing time "<<(ros::Time::now() - start_time));
    while (!viewer.wasStopped ()&& ros::ok())
    {
        viewer.spinOnce ();
        ROS_INFO("You are in the loop");
        ros::Duration(2.0).sleep();
    }
    return 0;
}
