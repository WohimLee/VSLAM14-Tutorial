&emsp;
# 4 实践：对极约束求解相机运动

下面，我们来练习一下如何通过 $Essential$ 矩阵求解相机运动。上一节实践部分的程序提供了特征匹配，而这次我们就使用匹配好的特征点来计算 $\pmb{E}，\pmb{F}$ 和 $\pmb{H}$，进而分解 $\pmb{E}$ 得到 $\pmb{R}, \pmb{t}$。整个程序使用 OpenCV 提供的算法进行求解。我们把上一节的特征提取封装成函数，以供后面使用。本节只展示位姿估计部分的代码。

&emsp;
>头文件
```c++
#ifndef MY_LIB_HPP
#define MY_LIB_HPP
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(
    const Mat& img_1, const Mat& img_2,
    std::vector<KeyPoint>& keypoints_1,
    std::vector<KeyPoint>& keypoints_2,
    std::vector< DMatch >& matches );

void pose_estimation_2d2d(
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector< DMatch > matches,
    Mat& R, Mat& t );

Point2d pixel2cam ( const Point2d& p, const Mat& K );

#endif
```

&emsp;
>库
```c++
opencv_calib3d
```

&emsp;
>函数：7.4_epipolar_geometry.cpp
```c++
#include "7.4_epipolar_geometry.hpp"

Point2d pixel2cam ( const Point2d& p, const Mat& K )
{
    return Point2d
           (
               ( p.x - K.at<double> ( 0,2 ) ) / K.at<double> ( 0,0 ),
               ( p.y - K.at<double> ( 1,2 ) ) / K.at<double> ( 1,1 )
           );
}


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
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    //-- 第一步:检测 Oriented FAST 角点位置
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    //-- 第二步:根据角点位置计算 BRIEF 描述子
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    //-- 第三步:对两幅图像中的BRIEF描述子进行匹配，使用 Hamming 距离
    vector<DMatch> match;
    //BFMatcher matcher ( NORM_HAMMING );
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

// 该函数提供了从特征点求解相机运动的部分，然后，我们在主函数中调用它，就能得到相机的运动
void pose_estimation_2d2d( 
    std::vector<KeyPoint> keypoints_1,
    std::vector<KeyPoint> keypoints_2,
    std::vector< DMatch > matches,
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
    Point2d principal_point ( 325.1, 249.7 );	//相机光心, TUM dataset标定值
    double focal_length = 521;			//相机焦距, TUM dataset标定值
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
```

&emsp;
>主函数 main.cpp
```c++
#include "7.4_epipolar_geometry.hpp"

int main(int argc, char** argv)
{
    //-- 读取图像
    Mat img_1 = imread("./chapter7/1.png", IMREAD_COLOR);
    Mat img_2 = imread("./chapter7/2.png", IMREAD_COLOR);
    if ( img_1.data == nullptr | img_2.data == nullptr) // 数据不存在，可能是路径错误
    {
        cerr << "检查文件路径" <<endl;
        return 0;
    }

    vector<KeyPoint> keypoints_1, keypoints_2;
    vector<DMatch> matches_optimized;
    find_feature_matches(img_1, img_2, keypoints_1, keypoints_2, matches_optimized);
    cout << "共找到 " << matches_optimized.size() << " 组匹配点" << endl;

    //-- 估计两张图像间运动
    Mat R,t;
    pose_estimation_2d2d( keypoints_1, keypoints_2, matches, R, t );

    //-- 验证 E=t^R*scale
    Mat t_x = (Mat_<double>(3,3) <<
    0, -t.at<double>(2,0), t.at<double>(1,0),
    t.at<double>(2,0), 0, -t.at<double>(0,0),
    -t.at<double>(1.0), t.at<double>(0,0), 0);

    cout<< "t^R=" <<endl<< t_x*R <<endl;

    //-- 验证对极约束
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    for ( DMatch m: matches )
    {
        Point2d pt1 = pixel2cam( keypoints_1[ m.queryIdx ].pt, K );
        Mat y1 = (Mat_<double>(3,1) << pt1.x, pt1.y, 1);
        Point2d pt2 = pixel2cam( keypoints_2[ m.trainIdx ].pt, K );
        Mat y2 = (Mat_<double>(3,1) << pt2.x, pt2.y, 1);
        Mat d = y2.t() * t_x * R * y1;
        cout << "epipolar constraint = " << d << endl;
    }

    return 0;
}
```

在程序的输出结果中可以看出，对极约束的满足精度约在 $10^{−3}$ 量级。根据前面的讨论，分解得到的 $\pmb{R}， \pmb{t}$ 一共有四种可能性。不过 OpenCV 替我们使用三角化检测角点的深度是否为正，从而选出正确的解

需要注意的地方是，我们要弄清程序求解出来的 $\pmb{R}， \pmb{t}$ 是什么意义。按照例程的定义，我们的对极约束是从

$$\pmb{x}_2 = \pmb{T}_{21}\pmb{x}_1$$

请读者在实践中务必清楚这里使用的变换顺序（因为有时我们会用 $\pmb{T}_{12}$），它们非常容易搞反

&emsp;
# 讨论

从演示程序中可以看到，输出的 $\pmb{E}$ 和 $\pmb{F}$ 之差相差了相机内参矩阵。虽然它们在数值上并不直观，但可以验证它们的数学关系。从 $\pmb{E}，\pmb{E}$ 和 $\pmb{H}$ 都可以分解出运动，不过 $\pmb{H}$ 需要假设特征点位于平面上。对于本实验的数据，这个假设是不好的，所以我们这里主要用 $\pmb{E}$ 来分解运动。

值得一提的是，由于 $\pmb{E}$ 本身具有尺度等价性，它分解得到的 $\pmb{t}，\pmb{R}$ 也有一个尺度等价性。而 $\pmb{R} ∈ SO(3)$ 自身具有约束，所以我们认为 t 具有一个尺度。换言之，在分解过程中，对 $\pmb{t}$ 乘以任意非零常数，分解都是成立的。因此，我们通常把 $\pmb{t}$ 进行归一化，让它的长度等于 $1$。

&emsp;
## 尺度不确定性
对 $\pmb{t}$ 长度的归一化，直接导致了单目视觉的尺 `度不确定性（Scale Ambiguity）`。

例如，程序中输出的 $\pmb{t}$ 第一维约 $0.822$。这个 $0.822$ 究竟是指 $0.822$ 米呢，还是 0.822 厘米呢，我们是没法确定的。因为对 $\pmb{t}$ 乘以任意比例常数后，对极约束依然是成立的。换言之，在单目 SLAM 中，对轨迹和地图同时缩放任意倍数，我们得到的图像依然是一样的。这在第二讲中就已经给读者介绍过了。

在单目视觉中，我们对两张图像的 $\pmb{t}$ 归一化，相当于固定了尺度。虽然我们不知道它的实际长度为多少，但我们以这时的 $t$ 为单位 $1$，计算相机运动和特征点的 3D 位置。这被称为单目 SLAM 的初始化。在初始化之后，就可以用 3D-2D 来计算相机运动了。初始化之后的轨迹和地图的单位，就是初始化时固定的尺度。因此，单目 SLAM 有一步不可避免的初始化。初始化的两张图像必须有一定程度的平移，而后的轨迹和地图都将以此步的平移为单位。

除了对 $\pmb{t}$ 进行归一化之外，另一种方法是令初始化时所有的特征点平均深度为 $1$，也可以固定一个尺度。相比于令 $\pmb{t}$ 长度为 $1$ 的做法，把特征点深度归一化可以控制场景的规模大小，使计算在数值上更稳定些。不过这并没有理论上的差别。

&emsp;
## 初始化的纯旋转问题
从 $\pmb{E}$ 分解到 $\pmb{t}，\pmb{R}$ 的过程中，如果相机发生的是纯旋转，导致 $\pmb{t}$ 为零，那么，得到的 $\pmb{E}$ 也将为零，这将导致我们无从求解 $\pmb{R}$。

不过，此时我们可以依靠 $\pmb{H}$ 求取旋转，但仅有旋转时，我们无法用三角测量估计特征点的空间位置（这将在下文提到），于是，另一个结论是，单目初始化不能只有纯旋转，必须要有一定程度的平移。如果没有平移，单目将无法初始化。

在实践当中，如果初始化时平移太小，会使得位姿求解与三角化结果不稳定，从而导致失败。相对的，如果把相机左右移动而不是原地旋转，就容易让单目 SLAM 初始化。因而有经验的 SLAM 研究人员，在单目 SLAM 情况下，经常选择让相机进行左右平移以顺利地进行初始化。

&emsp;
## 多于八对点的情况
当给定的点数多于八对时（比如例程找到了 79 对匹配），我们可以计算一个最小二乘
解。回忆式$（7.12）$中线性化后的对极约束
$$\begin{pmatrix}
u_1^1u_2^1， u_1^1v_2^1， u_1^1， v_1^1u_2^1， v_1^1v_2^1， v_1^1， u_2^1， v_2^1，1 \\ \\
u_1^2u_2^2， u_1^2v_2^2， u_1^2， v_1^2u_2^2， v_1^2v_2^2， v_1^2， u_2^2， v_2^2，1 \\ 
\vdots &  \\
u_1^8u_2^8， u_1^8v_2^8， u_1^8， v_1^8u_2^8， v_1^8v_2^8， v_1^8， u_2^8， v_2^8，1 
\end{pmatrix}
\begin{pmatrix}
e_1 \\ e_2 \\ e_3 \\ e_4 \\ e_5 \\ e_6 \\ e_7 \\ e_8 \\ e_9 
\end{pmatrix} = 0\quad (7.12)$$

我们把左侧的系数矩阵记为 $\pmb{A}$：
$$\pmb{Ae} = 0 $$

对于八点法，$\pmb{A}$ 的大小为 $8 × 9$。如果给定的匹配点多于 $8$，该方程构成一个超定方程，即不一定存在 $\pmb{e}$ 使得上式成立。因此，可以通过最小化一个二次型来求：
$$\min_e ∥\pmb{Ae}∥^2_2 = \min_e \pmb{e}^T \pmb{A}^T \pmb{Ae}$$

于是就求出了在最小二乘意义下的 $\pmb{E}$ 矩阵。不过，当可能存在误匹配的情况时，我们会更倾向于使用 `随机采样一致性（Random Sample Concensus, RANSAC）` 来求，而不是最小二乘。RANSAC 是一种通用的做法，适用于很多带错误数据的情况，可以处理带有错误匹配的数据。
