// 从给定的点云数据，给定视角生成深度图像
// 深度图像归根结底还是一幅图像
// 算法流程就是生成一个点云，然后定一个一个深度图像对象，设置一些参数，调用函数即可
#include <pcl/range_image/range_image.h>//深度图像头文件
#include "ros/ros.h"
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char** argv) 
{
  ros::init(argc, argv, "range_image_creation_pcl_tutorial");
  ros::NodeHandle nh;
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
  // 生成一个矩形点云
  for (float y=-0.5f; y<=0.5f; y+=0.01f) 
  {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) 
    {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.points.push_back(point);
    }
  }
  //声明为无序点云
  pointCloud.width = (uint32_t) pointCloud.points.size();
  pointCloud.height = 1;
  
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  
  // 角分辨率设置为1度，即深度图像中一个像素对应的角度大小（因为这种是球投影的方式，所以就有了投影的角度一说）
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
  // 360度表示模拟的传感器对周围的环境有一个完整的视角，但是相对应的存储资源也会增加。
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  
  // 180.0 degree in radians
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  // sensorPose定义模拟深度相机的六自由度位置，默认是一个4*4的单位阵，也就是从原点观察
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
  // CAMERA_FRAME 说明Z轴是朝前的，同时也可以选择LASER_FRAME，此时Z轴向下。
  float noiseLevel=0.00;
  // 使用一个归一化的Z缓冲器来创建深度图像，如果想让邻近点集路落在同一个像素单元，可以设置一个较高的值，比如0.05，可以理解为深度距离值是通过查询点半径为5cm的圆内包含的点计算平均值得到。
  float minRange = 0.0f;
  // 如果minRange大于0， 则所有模拟器所在位置半径minRange内的邻近点将被忽略，视为盲区。
  int borderSize = 1;
  // 裁剪图像的时候，如果boardSize>0， 将在图像周围留下当前视点不可见点的边界。
  
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                  sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
  
  std::cout << rangeImage << "\n";

  // 深度图像共有三种类型的点集，有效点集是距离大于零的点集，当前视点不可见的点集x = y = z = NAN , 且值域为负无穷大，远距离点集x = y = z = NAN 且其值域为无穷大
}