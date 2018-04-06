// 八叉树一般也是可以用来进行空间搜索，但是pcl的octree还集成了一些其他的功能
// 这个节点主要是利用了八叉树的搜索功能，实现了体素内的点搜索，k邻近搜索，以及指定半径内的搜索

#include <pcl/point_cloud.h>
#include "pcl/point_types.h"
#include <pcl/octree/octree.h>
#include <iostream>
#include <vector>
#include <ctime>
#include "ros/ros.h"

int main (int argc, char**argv)
{
    ros::init(argc, argv, "octree_pcl_tutorial");
    ros::NodeHandle nh;
    srand ((unsigned int) time (NULL));
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->width =1000;
    cloud->height =1;
    cloud->points.resize (cloud->width * cloud->height);
    for (size_t i=0; i< cloud->points.size (); ++i)
    {
        cloud->points[i].x =1024.0f* rand () / (RAND_MAX +1.0f);
        cloud->points[i].y =1024.0f* rand () / (RAND_MAX +1.0f);
        cloud->points[i].z =1024.0f* rand () / (RAND_MAX +1.0f);
    }

    float resolution =128.0f;
    //通过设置分辨率来初始化八叉树,最低一级单个体素的大小(注意一个体素并不是一个点的大小，一个体素是指最小的一个方块的尺寸)，代表的是空间信息，这样根据分辨率和点云维数就可以确定深度
    //该octree用它的叶节点存放点索引向量，该分辨率参数描述最低一级octree最小体素的尺寸
    //因此octree的深度是分辨率和点云空间维数的函数。
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud (cloud);
    octree.addPointsFromInputCloud ();
    pcl::PointXYZ searchPoint;
    searchPoint.x=1024.0f* rand () / (RAND_MAX +1.0f);
    searchPoint.y=1024.0f* rand () / (RAND_MAX +1.0f);
    searchPoint.z=1024.0f* rand () / (RAND_MAX +1.0f);

    //体素内近邻搜索
    std::vector<int>pointIdxVec;
    // 存储的是点的下标
    if (octree.voxelSearch (searchPoint, pointIdxVec))
    {
        std::cout<<"Neighbors within voxel search at ("<<searchPoint.x
        <<" "<<searchPoint.y
        <<" "<<searchPoint.z<<")"
        <<std::endl;
        for (size_t i=0; i<pointIdxVec.size (); ++i)
            std::cout<<"    "<< cloud->points[pointIdxVec[i]].x 
            <<" "<< cloud->points[pointIdxVec[i]].y 
            <<" "<< cloud->points[pointIdxVec[i]].z <<std::endl;
    }
    //K近邻搜索
    int K =10;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::cout<<"K nearest neighbor search at ("<<searchPoint.x
        <<" "<<searchPoint.y
        <<" "<<searchPoint.z
        <<") with K="<< K <<std::endl;
    if (octree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >0)
    {
        for (size_t i=0; i<pointIdxNKNSearch.size (); ++i)
        std::cout<<"    "<<   cloud->points[ pointIdxNKNSearch[i] ].x 
            <<" "<< cloud->points[pointIdxNKNSearch[i] ].y 
            <<" "<< cloud->points[pointIdxNKNSearch[i] ].z 
            <<" (squared distance: "<<pointNKNSquaredDistance[i] <<")"<<std::endl;
    }
    //半径内近邻搜索
    std::vector<int>pointIdxRadiusSearch;
    std::vector<float>pointRadiusSquaredDistance;
    float radius =256.0f* rand () / (RAND_MAX +1.0f);
    std::cout<<"Neighbors within radius search at ("<<searchPoint.x
    <<" "<<searchPoint.y
    <<" "<<searchPoint.z
    <<") with radius="<< radius <<std::endl;
    if (octree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0)
    {
    for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i)
    std::cout<<"    "<<   cloud->points[ pointIdxRadiusSearch[i] ].x 
        <<" "<< cloud->points[pointIdxRadiusSearch[i] ].y 
        <<" "<< cloud->points[pointIdxRadiusSearch[i] ].z 
        <<" (squared distance: "<<pointRadiusSquaredDistance[i] <<")"<<std::endl;
    }
}
