// 使用filter模块中的voxel降采样，核心思想是指定一个体素的大小（一般比较小），然后将一个体素内点的均值当做一个值
// 这样既达到了降采样的目的，也可以保证点的正确性

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "ros/ros.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "voxel_grid_pcl_tutorial");
  ros::NodeHandle nh;
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  
  pcl::io::loadPCDFile("/home/leon/graspdemo/src/pcl_tutorial/table_scene_lms400.pcd", *cloud); 

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  return (0);
}
















// #include <iostream>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/filters/voxel_grid.h>
// #include "ros/ros.h"
// #include "sensor_msgs/PointCloud2.h"

// int main (int argc, char** argv)
// {
//   ros::init(argc, argv, "voxel_grid_pcl_tutorial");
//   ros::NodeHandle nh;
//   pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
//   pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//   pcl::PCLPointCloud2 cloud_filtered;
//   // 点云读取对象
//   pcl::PCDReader reader;
//   // 设置需要过滤的点云给滤波对象
//   reader.read ("/home/leon/graspdemo/src/pcl_tutorial/table_scene_lms400.pcd", *cloud); // 读取点云
//   std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
//        << " data points (" << pcl::getFieldsList (*cloud) << ").";
//   // 创建滤波对象
//   pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//   sor.setInputCloud (cloudPtr);//设置需要过滤的点云给滤波对象
//   sor.setLeafSize (0.01f, 0.01f, 0.01f);//设置滤波的体素为1立方厘米
//   sor.filter (cloud_filtered);//执行滤波，存储输出到cloud_fitered
//   std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
//        << " data points (" << pcl::getFieldsList (cloud_filtered) << ").";
//   pcl::PCDWriter writer;//将滤波后的数据存储起来。
//   writer.write ("table_scene_lms400_downsampled.pcd", cloud_filtered, 
//          Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
//   return (0);
// }
