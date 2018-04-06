// 使用统计分析的方法来去除离群点
// 核心思想是对于点云中的每一个点，计算其到其附近K（需要自己指定）个点的距离分布，得到均值和方差，并且近似高斯分布
// 如果平均距离在标准距离之外（由全局平均距离和方差计算），则认为这个点是离群点

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "ros/ros.h"

int main (int argc, char** argv)
{
  ros::init(argc, argv, "stastical_removal_pcl_tutorial");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/leon/graspdemo/src/pcl_tutorial/table_scene_lms400.pcd", *cloud);
  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;//输出点云头的信息

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//创建滤波器对象
  sor.setInputCloud (cloud);                        //设置待滤波的点云
  sor.setMeanK (50);                                //设置在进行统计时考虑查询点临近点数
  sor.setStddevMulThresh (2.0);                     //设置判断是都为离群点的阈值，此处的意思是2个标准差
  sor.filter (*cloud_filtered);                     //执行滤波并保存在cloud_filtered

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_inliers.pcd", *cloud_filtered, false);
  // 有了下面的这句话，实际上是计算剔除的点
  sor.setNegative (true);
  sor.filter (*cloud_filtered);
  writer.write<pcl::PointXYZ> ("table_scene_lms400_outliers.pcd", *cloud_filtered, false);
  return (0);
}