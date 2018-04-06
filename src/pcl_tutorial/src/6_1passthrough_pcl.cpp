// 使用直通滤波器对原始的点云数据进行处理，指定一个滤波方向，在某一个范围内的数据被剔除或者保存下来

#include <iostream>
#include <ctime>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include "ros/ros.h"

int main (int argc, char** argv)
{ 
  ros::init(argc, argv, "passthrough_pcl");
  ros::NodeHandle nh;
  srand(time(0));//随机生成数据，如果这句话起作用的话，每次的随机数都是不一样的。
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  //生成并填充点云数据
  cloud->width  = 5;
  //无序点云
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);
  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = rand () / (RAND_MAX + 1.0f)-0.5;
    cloud->points[i].y = rand () / (RAND_MAX + 1.0f)-0.5;
    cloud->points[i].z = rand () / (RAND_MAX + 1.0f)-0.5;
  }
  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
                        << cloud->points[i].y << " " 
                        << cloud->points[i].z << std::endl;
  // 直通滤波器
  pcl::PassThrough<pcl::PointXYZ> pass;  //设置滤波器对象
  pass.setInputCloud (cloud);            //设置输入点云
  pass.setFilterFieldName ("z");         //设置过滤时所需要的点云类型的z字段
  pass.setFilterLimits (0.0, 1.0);       //设置在过滤字段上的范围，在该范围内的数据都被保留下来
  //pass.setFilterLimitsNegative (true); //这句话起作用的时候，在上述设置的范围外的数据都被保存下来
  pass.filter (*cloud_filtered);         //执行滤波，保存过滤结果在cloud_filtered

  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
                        << cloud_filtered->points[i].y << " " 
                        << cloud_filtered->points[i].z << std::endl;
  return (0);
}
