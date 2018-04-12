// 使用积分图计算一个有序点云的法线，并且积分图只适用于有序点云
// 

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/cloud_viewer.h>
#include "ros/ros.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "normal_estimation_using_internal_image");
    ros::NodeHandle nh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile ("/home/leon/graspdemo/src/pcl_tutorial/table_scene_mug_stereo_textured.pcd", *cloud);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;      // 积分估计法线对象
    ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);                  // 设置估计方法，另外还有协方差法等
    ne.setMaxDepthChangeFactor(0.02f);                                      // 最大深度变化系数
    ne.setNormalSmoothingSize(10.0f);                                       // 优化法线方向时考虑邻域大小
    ne.setInputCloud(cloud);                                                // 输入点云，必须是有序点云
    ne.compute(*normals);                                                   // 执行法线估计，存出结果到normals

    // 以下代码完成可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor (0.0, 0.0, 0.5);
    viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normals);
    while (!viewer.wasStopped () && ros::ok())
    {
        viewer.spinOnce ();
    }
    return 0;
}
