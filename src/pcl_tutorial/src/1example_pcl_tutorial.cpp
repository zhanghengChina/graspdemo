// 第一个PCL节点，主要是测试一下可行性,以及和ROS建立联系
// 代码的实际功能是对得到的点云进行降采样
// pcl的点云格式都可以通过ros的topic进行发布，参考链接http://wiki.ros.org/pcl/Overview
// 需要注意的是这个节点里面实现了几乎常用的数据格式之间的转换

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "sensor_msgs/Image.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  // sensor_msgs::PointCloud2 --> pcl::PCLPointCloud2
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);//设置滤波时创建的体素体积为1立方厘米
  sor.filter (cloud_filtered);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);
  // pcl::PCLPointCloud2 --> sensor_msgs::PointCloud2
  pub.publish (output);
  ROS_INFO("You are in the callback function!");
  pcl::PointCloud<pcl::PointXYZ> point_cloud_template;
  pcl::fromPCLPointCloud2(*cloud,point_cloud_template);
  // pcl::PCLPointCloud2 --> pcl::PointCloud<pcl::PointXYZ>
  pcl::PointCloud<pcl::PointXYZRGB> point_cloud_format;
  sensor_msgs::Image ros_image;
  pcl::fromROSMsg(*cloud_msg,point_cloud_format);
  // sensor_msgs::PointCloud2 --> pcl::PointCloud<pcl::PointXYZ>
  pcl::toROSMsg(point_cloud_format,ros_image);   
  // pcl::PointCloud<pcl::PointXYZ> --> sensor_msgs::Image
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth_registered/points", 1, cloud_cb);
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  ros::spin ();
}