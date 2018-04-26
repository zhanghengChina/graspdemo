#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace cv;
using namespace std;

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "surf");
    ros::NodeHandle nh;
	Mat trainImage = imread("/home/leon/graspdemo/src/opencvtest/src/surf1.jpg") ,trainImage_gray;
	imshow("训练图像",trainImage);
	cvtColor(trainImage, trainImage_gray, CV_BGR2GRAY);
	vector<KeyPoint> train_keyPoint;
	Mat trainDescriptor;
	SurfFeatureDetector featureDetector(80);
	featureDetector.detect(trainImage_gray, train_keyPoint);
	SurfDescriptorExtractor featureExtractor;
	featureExtractor.compute(trainImage_gray, train_keyPoint, trainDescriptor);
	FlannBasedMatcher matcher;
	vector<Mat> train_desc_collection(1, trainDescriptor);
	matcher.add(train_desc_collection);
	matcher.train();

    Mat testImage = imread("/home/leon/graspdemo/src/opencvtest/src/surf2.jpg", 1 ), testImage_gray;
	cvtColor(testImage, testImage_gray, CV_BGR2GRAY);
	vector<KeyPoint> test_keyPoint;
	Mat testDescriptor;
	featureDetector.detect(testImage_gray, test_keyPoint);
	featureExtractor.compute(testImage_gray, test_keyPoint, testDescriptor);
	vector<vector<DMatch> > matches;
	matcher.knnMatch(testDescriptor, matches, 2);
	vector<DMatch> goodMatches;
	for(unsigned int i = 0; i < matches.size(); i++)
	{
		if(matches[i][0].distance < 0.6 * matches[i][1].distance)
			goodMatches.push_back(matches[i][0]);
	}

	Mat dstImage;
	drawMatches(testImage, test_keyPoint, trainImage, train_keyPoint, goodMatches, dstImage);
	imshow("匹配图像", dstImage);
    waitKey();
    return 0;
}


// #include "ros/ros.h"

// #include "opencv2/core/core.hpp"
// #include "opencv2/features2d/features2d.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include <opencv2/nonfree/nonfree.hpp>
// #include<opencv2/legacy/legacy.hpp>
// #include <iostream>
// using namespace cv;
// using namespace std;



// int main( int argc, char** argv )
// {
//     ros::init(argc, argv, "surf");
//     ros::NodeHandle nh;
// 	Mat img_1 = imread("/home/leon/graspdemo/src/opencvtest/src/surf1.jpg", 1 );
// 	Mat img_2 = imread("/home/leon/graspdemo/src/opencvtest/src/surf2.jpg", 1 );
// 	if( !img_1.data || !img_2.data ) { printf("读取文件错误 \n"); return false; }  

// 	int minHessian = 300;
// 	SURF detector( minHessian );
// 	std::vector<KeyPoint> keypoints_1, keypoints_2;
// 	detector.detect( img_1, keypoints_1 );
// 	detector.detect( img_2, keypoints_2 );

// 	SURF extractor;
// 	Mat descriptors_1, descriptors_2;
// 	extractor.compute( img_1, keypoints_1, descriptors_1 );
// 	extractor.compute( img_2, keypoints_2, descriptors_2 );

// 	FlannBasedMatcher matcher;
// 	std::vector< DMatch > matches;
// 	matcher.match( descriptors_1, descriptors_2, matches );
// 	double max_dist = 0; double min_dist = 100;
// 	for( int i = 0; i < descriptors_1.rows; i++ )
// 	{
// 		double dist = matches[i].distance;
// 		if( dist < min_dist ) min_dist = dist;
// 		if( dist > max_dist ) max_dist = dist;
// 	}
// 	printf("最大值 : %f \n", max_dist );
// 	printf("最小值: %f \n", min_dist );
// 	std::vector< DMatch > good_matches;
// 	for( int i = 0; i < descriptors_1.rows; i++ )
// 	{ 
// 		if( matches[i].distance < 2*min_dist )
// 		{ good_matches.push_back( matches[i]); }
// 	}
// 	Mat img_matches;
// 	drawMatches( img_1, keypoints_1, img_2, keypoints_2,
// 		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
// 		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

// 	// for( int i = 0; i < good_matches.size(); i++ )
// 	// { printf( ">����������ƥ��� [%d] ������1: %d  -- ������2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); }
// 	imshow( "匹配图像", img_matches );
// 	waitKey(0);
// 	return 0;
// }








// #include "opencv2/core/core.hpp"
// #include "opencv2/features2d/features2d.hpp"
// #include "opencv2/highgui/highgui.hpp"
// #include "opencv2/nonfree/nonfree.hpp"
// #include "opencv2/legacy/legacy.hpp" 
// #include "opencv2/opencv.hpp" 
// #include <iostream>
// #include "ros/ros.h"
// using namespace cv;


// int main( int argc, char** argv )
// {
//     ros::init(argc, argv, "surf");
//     ros::NodeHandle nh;

// 	Mat srcImage1 = imread("1.jpg", 1 );
// 	Mat srcImage2 = imread("2.jpg", 1 );
// 	if( !srcImage1.data || !srcImage2.data )
// 	{ printf("读取文件错误 \n"); return false; } 
// 	imshow("第一张图",srcImage1);
// 	imshow("第二章图",srcImage2);

// 	int minHessian = 400;
// 	cv::SurfFeatureDetector detector( minHessian );
// 	std::vector<KeyPoint> keypoints_1, keypoints_2;

// 	detector.detect( srcImage1, keypoints_1 );
// 	detector.detect( srcImage2, keypoints_2 );


// 	Mat img_keypoints_1; Mat img_keypoints_2;
// 	drawKeypoints( srcImage1, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
// 	drawKeypoints( srcImage2, keypoints_2, img_keypoints_2, Scalar::all(-1), DrawMatchesFlags::DEFAULT );


// 	imshow("包含Surf的第一张图", img_keypoints_1 );
// 	imshow("包含Surf的第二章图", img_keypoints_2 );

// 	waitKey(0);
// 	return 0;
// }
