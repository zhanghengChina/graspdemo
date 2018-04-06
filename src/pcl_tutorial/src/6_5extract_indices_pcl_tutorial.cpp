// 从一个点云中提取索引
// 因为牵涉到点云分割的内容，没太看懂

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include "ros/ros.h"

int main (int argc, char** argv)
{
  ros::init(argc,argv,"extract_indices_pcl_tutorial");
  ros::NodeHandle nh;

  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), 
                                                                                           cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::io::loadPCDFile("/home/leon/graspdemo/src/pcl_tutorial/table_scene_lms400.pcd", *cloud_blob);
  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;

  // 降采样
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);

  // 目前看来好像就这个降采样是使用pcl::PointCloud2数据格式。
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ()); //创建模型系数
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());         
  // 点云分割模块       
  pcl::SACSegmentation<pcl::PointXYZ> seg;                                  //创建分割对象
  seg.setOptimizeCoefficients (true);                                       //设置对估计的模型参数进行优化处理
  seg.setModelType (pcl::SACMODEL_PLANE);                                   //设置分割模型类别
  seg.setMethodType (pcl::SAC_RANSAC);                                      //设置用哪个随机参数估计方法
  seg.setMaxIterations (1000);                                              //设置最大迭代次数
  seg.setDistanceThreshold (0.01);                                          //设置判断是否为模型内点的距离阈值

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;                               //创建点云提取对象
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);                                     //设置输入点云
    seg.segment (*inliers, *coefficients);                                  
    if (inliers->indices.size () == 0)      
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }
    std::cout<<inliers->indices.size()<<std::endl;

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    
    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }
  return (0);
}