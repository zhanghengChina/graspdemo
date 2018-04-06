// 这个节点主要是学习PCL中kdtree的使用；
// kd树的核心作用还是空间搜索，在这个节点中使用了k邻近搜索和半径搜索

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include "ros/ros.h"
#include <pcl/visualization/cloud_viewer.h>

int main (int argc, char**argv)
{
    ros::init(argc, argv, "kd_tree_search_pcl_tutorial");
    ros::NodeHandle nh;
    srand (time (NULL));//使用系统时间初始化随机函数种子，这样每次程序运行的时候产生的随机数都是不一样的。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width =1000;
    cloud->height =1;
    cloud->points.resize (cloud->width * cloud->height);
    for (size_t i=0; i< cloud->points.size (); ++i)
    {
        cloud->points[i].x =1024.0f* rand () / (RAND_MAX +1.0f);
        cloud->points[i].y =1024.0f* rand () / (RAND_MAX +1.0f);
        cloud->points[i].z =1024.0f* rand () / (RAND_MAX +1.0f);
    }

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    pcl::PointXYZ searchPoint;
    searchPoint.x=1024.0f* rand () / (RAND_MAX +1.0f);
    searchPoint.y=1024.0f* rand () / (RAND_MAX +1.0f);
    searchPoint.z=1024.0f* rand () / (RAND_MAX +1.0f);

    int K =10;
    std::vector<int> pointIdxNKNSearch(K);//括号里面的10代表的是这个容器里面有是个元素
    std::vector<float> pointNKNSquaredDistance(K);
    std::cout<<"K nearest neighbor search at ("<<searchPoint.x
    <<" "<<searchPoint.y
    <<" "<<searchPoint.z
    <<") with K="<< K <<std::endl;
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >0 )
    {
    for (size_t i=0; i<pointIdxNKNSearch.size (); ++i)
    std::cout<<"    "<<   cloud->points[ pointIdxNKNSearch[i] ].x 
    <<" "<< cloud->points[pointIdxNKNSearch[i] ].y 
    <<" "<< cloud->points[pointIdxNKNSearch[i] ].z 
    <<" (squared distance: "<<pointNKNSquaredDistance[i] <<")"<<std::endl;
    }

    //半径内近邻搜索
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float radius =256.0f* rand () / (RAND_MAX +1.0f);
    std::cout<<"Neighbors within radius search at ("<<searchPoint.x
    <<" "<<searchPoint.y
    <<" "<<searchPoint.z
    <<") with radius="<< radius <<std::endl;
    if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) >0 )
    {
    for (size_t i=0; i<pointIdxRadiusSearch.size (); ++i)
    std::cout<<"    "<<   cloud->points[ pointIdxRadiusSearch[i] ].x 
    <<" "<< cloud->points[pointIdxRadiusSearch[i] ].y 
    <<" "<< cloud->points[pointIdxRadiusSearch[i] ].z 
    <<" (squared distance: "<<pointRadiusSquaredDistance[i] <<")"<<std::endl;
    }



    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud (cloud);
    while (!viewer.wasStopped ()&&ros::ok())
    {
        ROS_INFO("Waiting for stopping!");
        ros::Duration(2.0).sleep();
    }
    return 0;
}
