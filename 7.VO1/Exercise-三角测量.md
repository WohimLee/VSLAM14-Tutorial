&emsp;
# 6 实践：三角测量

下面，我们演示如何根据之前根据对极几何求解的相机位姿，通过三角化求出上一节特征点的空间位置。我们调用 OpenCV 提供的 triangulation 函数进行三角化。

&emsp;
>头文件
```c++
#ifndef TRIANGULATION_HPP
#define TRIANGULATION_HPP
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void find_feature_matches (
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

void pose_estimation_2d2d (
    const std::vector<KeyPoint>& keypoints_1,
    const std::vector<KeyPoint>& keypoints_2,
    const std::vector< DMatch >& matches,
    Mat& R, Mat& t );

void triangulation (
    const vector<KeyPoint>& keypoint_1,
    const vector<KeyPoint>& keypoint_2,
    const std::vector< DMatch >& matches,
    const Mat& R, const Mat& t,
    vector<Point3d>& points
);

// 像素坐标转相机归一化坐标
Point2f pixel2cam( const Point2d& p, const Mat& K );

#endif
```

&emsp;
>7.6_triangulation.cpp
```c++
#include "7.6_triangulation.hpp"

// 像素坐标转相机归一化坐标
Point2f pixel2cam( const Point2d& p, const Mat& K );



void find_feature_matches ( const Mat& img_1, const Mat& img_2,
                            std::vector<KeyPoint>& keypoints_1,
                            std::vector<KeyPoint>& keypoints_2,
                            std::vector< DMatch >& matches )
{
    //-- 初始化
    Mat descriptors_1, descriptors_2;
    // used in OpenCV3 
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();
    // use this if you are in OpenCV2 
    // Ptr<FeatureDetector> detector = FeatureDetector::create ( "ORB" );
    // Ptr<DescriptorExtractor> descriptor = DescriptorExtractor::create ( "ORB" );
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create("BruteForce-Hamming");
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
   // BFMatcher matcher ( NORM_HAMMING );
    matcher->match ( descriptors_1, descriptors_2, match );

    //-- 第四步:匹配点对筛选
    double min_dist=10000, max_dist=0;

    //找出所有匹配之间的最小距离和最大距离, 即是最相似的和最不相似的两组点之间的距离
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        double dist = match[i].distance;
        if ( dist < min_dist ) min_dist = dist;
        if ( dist > max_dist ) max_dist = dist;
    }

    printf ( "-- Max dist : %f \n", max_dist );
    printf ( "-- Min dist : %f \n", min_dist );

    //当描述子之间的距离大于两倍的最小距离时,即认为匹配有误.但有时候最小距离会非常小,设置一个经验值30作为下限.
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( match[i].distance <= max ( 2*min_dist, 30.0 ) )
        {
            matches.push_back ( match[i] );
        }
    }
}

void pose_estimation_2d2d (
    const std::vector<KeyPoint>& keypoints_1,
    const std::vector<KeyPoint>& keypoints_2,
    const std::vector< DMatch >& matches,
    Mat& R, Mat& t )
{
    // 相机内参,TUM Freiburg2
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );

    //-- 把匹配点转换为vector<Point2f>的形式
    vector<Point2f> points1;
    vector<Point2f> points2;

    for ( int i = 0; i < ( int ) matches.size(); i++ )
    {
        points1.push_back ( keypoints_1[matches[i].queryIdx].pt );
        points2.push_back ( keypoints_2[matches[i].trainIdx].pt );
    }

    //-- 计算基础矩阵
    Mat fundamental_matrix;
    fundamental_matrix = findFundamentalMat ( points1, points2, FM_8POINT );
    cout<<"fundamental_matrix is "<<endl<< fundamental_matrix<<endl;

    //-- 计算本质矩阵
    Point2d principal_point ( 325.1, 249.7 );				//相机主点, TUM dataset标定值
    int focal_length = 521;						//相机焦距, TUM dataset标定值
    Mat essential_matrix;
    essential_matrix = findEssentialMat ( points1, points2, focal_length, principal_point );
    cout<<"essential_matrix is "<<endl<< essential_matrix<<endl;

    //-- 计算单应矩阵
    Mat homography_matrix;
    homography_matrix = findHomography ( points1, points2, RANSAC, 3 );
    cout<<"homography_matrix is "<<endl<<homography_matrix<<endl;

    //-- 从本质矩阵中恢复旋转和平移信息.
    recoverPose ( essential_matrix, points1, points2, R, t, focal_length, principal_point );
    cout<<"R is "<<endl<<R<<endl;
    cout<<"t is "<<endl<<t<<endl;
}

void triangulation ( 
    const vector< KeyPoint >& keypoint_1, 
    const vector< KeyPoint >& keypoint_2, 
    const std::vector< DMatch >& matches,
    const Mat& R, const Mat& t, 
    vector< Point3d >& points )
{
    Mat T1 = (Mat_<float> (3,4) <<
        1,0,0,0,
        0,1,0,0,
        0,0,1,0);
    Mat T2 = (Mat_<float> (3,4) <<
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t.at<double>(0,0),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t.at<double>(1,0),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t.at<double>(2,0)
    );
    
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    vector<Point2f> pts_1, pts_2;
    for ( DMatch m:matches )
    {
        // 将像素坐标转换至相机坐标
        pts_1.push_back ( pixel2cam( keypoint_1[m.queryIdx].pt, K) );
        pts_2.push_back ( pixel2cam( keypoint_2[m.trainIdx].pt, K) );
    }
    
    Mat pts_4d;
    cv::triangulatePoints( T1, T2, pts_1, pts_2, pts_4d );
    
    // 转换成非齐次坐标
    for ( int i=0; i<pts_4d.cols; i++ )
    {
        Mat x = pts_4d.col(i);
        x /= x.at<float>(3,0); // 归一化
        Point3d p (
            x.at<float>(0,0), 
            x.at<float>(1,0), 
            x.at<float>(2,0) 
        );
        points.push_back( p );
    }
}

Point2f pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2f
    (
        ( p.x - K.at<double>(0,2) ) / K.at<double>(0,0), 
        ( p.y - K.at<double>(1,2) ) / K.at<double>(1,1) 
    );
}
```

&emsp;
>主函数 main.cpp
```c++

#include "7.6_triangulation.hpp"

int main ( int argc, char** argv )
{

    //-- 读取图像
    Mat img_1 = imread ( "./chapter7/1.png", IMREAD_COLOR );
    Mat img_2 = imread ( "./chapter7/2.png", IMREAD_COLOR );

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches;
    find_feature_matches ( img_1, img_2, keypoints_1, keypoints_2, matches );
    cout<<"一共找到了"<<matches.size() <<"组匹配点"<<endl;

    //-- 估计两张图像间运动
    Mat R,t;
    pose_estimation_2d2d ( keypoints_1, keypoints_2, matches, R, t );

    //-- 三角化
    vector<Point3d> points;
    triangulation( keypoints_1, keypoints_2, matches, R, t, points );
    
    //-- 验证三角化点与特征点的重投影关系
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for ( int i=0; i<matches.size(); i++ )
    {
        Point2d pt1_cam = pixel2cam( keypoints_1[ matches[i].queryIdx ].pt, K );
        Point2d pt1_cam_3d(
            points[i].x/points[i].z, 
            points[i].y/points[i].z 
        );
        
        cout<<"point in the first camera frame: "<<pt1_cam<<endl;
        cout<<"point projected from 3D "<<pt1_cam_3d<<", d="<<points[i].z<<endl;
        
        // 第二个图
        Point2f pt2_cam = pixel2cam( keypoints_2[ matches[i].trainIdx ].pt, K );
        Mat pt2_trans = R*( Mat_<double>(3,1) << points[i].x, points[i].y, points[i].z ) + t;
        pt2_trans /= pt2_trans.at<double>(2,0);
        cout<<"point in the second camera frame: "<<pt2_cam<<endl;
        cout<<"point reprojected from second frame: "<<pt2_trans.t()<<endl;
        cout<<endl;
    }
    
    return 0;
}
```

&emsp;
# 讨论

关于三角测量，还有一个必须注意的地方。

三角测量是由平移得到的，有平移才会有对极几何中的三角形，才谈的上三角测量。因此，纯旋转是无法使用三角测量的，因为对极约束将永远满足。在平移存在的情况下，我们还要关心三角测量的不确定性，这会引出一个三角测量的矛盾。

<div align="center">
    <image src="./imgs/7.6-1.png" width = 500>
</div>
&emsp;

- 当平移 $\pmb{t}$ 很小时：像素上的不确定性将导致较大的深度不确定性。也就是说，如果特征点运动一个像素 $δx$，使得视线角变化了一个角度 $δθ$，那么测量到深度值将有 $δd$ 的变化
- 当平移 $\pmb{t}$ 较大时：$δd$ 将明显变小，这说明平移较大时，在同样的相机分辨率下，三角化测量将更精确。对该过程的定量分析可以使用正弦定理得到，但我们这里先考虑定性分析。

&emsp;
>增加三角化的精度的方法
- 提高特征点的提取精度，也就是提高图像分辨率——但这会导致图像变大，提高计算成本
- 使平移量增大。但是，平移量增大，会导致图像的外观发生明显的变化，比如箱子原先被挡住的侧面显示出来了，比如反射光发生变化了，等等。外观变化会使得特征提取与匹配变得困难

总而言之，在增大平移，会导致匹配失效；而平移太小，则三角化精度不够——这就是三角化的矛盾。


&emsp;
>深度滤波器（Depth Filter）的引出
- 虽然本节只介绍了三角化的深度估计，但只要我们愿意，也能够定量地计算每个特征点的位置及不确定性。
    
    所以，如果假设特征点服从高斯分布，并且对它不断地进行观测，在信息正确的情况下，我们就能够期望它的方差会不断减小乃至收敛。这就得到了一个滤波器，称为`深度滤波器（Depth Filter）`。不过，由于它的原理较复杂，我们留到第 13 讲再详细讨论它。


下面，我们来讨论从 3D-2D 的匹配点来估计相机运动，以及 3D-3D 的估计方法。