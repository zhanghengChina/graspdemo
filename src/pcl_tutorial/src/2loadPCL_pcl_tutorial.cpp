// 读取点云信息并显示
// 基本上读取离线点云数据一个函数就可以搞定所有的数据类型

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
using namespace std;

int main (int argc, char** argv)
{
  ros::init (argc, argv, "loadPCDFile_pcl_tutorial");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // 下面这个加载方法比较通用，当然也有其他的加载方法，但是知道一个就够用了
  if(pcl::io::loadPCDFile<pcl::PointXYZ>("/home/leon/graspdemo/src/pcl_tutorial/room_scan1.pcd",*cloud) == -1)
  {
      ROS_ERROR("Can't load PCD file");
      return(-1);
  }
  ROS_INFO("Loaded PCD file successfully!");

  pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
  std::cout<<"the point number is "<<cloud->width*cloud->height<<std::endl;
  viewer.showCloud (cloud);
  //显示直到程序结束，并且不占用主线程
  while (!viewer.wasStopped ()&&ros::ok())
  {
    ROS_INFO("You are in the loop now!");
    ros::Duration(2.0).sleep();
  }
  return(0);
}