&emsp;
# 2 实践：特征提取和匹配

<div align="center">
    <image src="./imgs/7.2-1.png" width = 600>
</div>
&emsp;

目前主流的几种图像特征在 OpenCV 开源图像库中都已经集成完毕，我们可以很方便地进行调用。下面我们来实际练习一下 OpenCV 的图像特征提取、计算和匹配的过程。

我们为此实验准备了两张图像，位于 slambook/ch7/下的 1.png 和 2.png，如图 7-5 所示。

它们是来自公开数据集 `[37]A benchmark for the evaluation of rgb-d SLAM systems` 中的两张图像，我们看到相机发生了微小的运动。

本节演示如何提取 ORB 特征，并进行匹配。下个程序将演示如何估计相机运动。

>7.2_features_matching.hpp
```c++
#ifndef FEATURES_MATCHING_HPP
#define FEATURES_MATCHING_HPP
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

void find_feature_matches(
    cv::Mat img_1, cv::Mat img_2,
    std::vector<KeyPoint> keypoints_1, 
    std::vector<KeyPoint> keypoints_2,
    std::vector<DMatch> matches
);

#endif // FEATURES_MATCHING_HPP
```

&emsp;
>库
```c++
opencv_features2d
```

<div align="center">
    <image src="./imgs/7.2-2.png" width = 600>
</div>
&emsp;
<div align="center">
    <image src="./imgs/7.2-3.png" width = 600>
</div>
&emsp;


&emsp;
>7.2_features_matching.cpp
```c++
#include "7.2_features_matching.hpp"

void find_feature_matches(
    cv::Mat img_1, cv::Mat img_2,
    std::vector<KeyPoint> keypoints_1, 
    std::vector<KeyPoint> keypoints_2,
    std::vector<DMatch> matches_optimized
)
{
    //-- 初始化
    cv::Ptr<ORB> orb = cv::ORB::create(
        500, 1.2f, 8, 31, 0, 2, ORB::HARRIS_SCORE,31,20);
    
    //-- 第一步：检测 Oriented FAST 角点位置
    orb->detect(img_1, keypoints_1);
    orb->detect(img_2, keypoints_2);
    // 画出检测出来的关键点（只画了img_1）
    cv::Mat show;
    drawKeypoints(
        img_1, keypoints_1, show, 
        cv::Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    cv::imshow("OFAST_points_img1", show);
    cv::waitKey(0);
    //-- 第二步：根据角点位置计算 BRIEF 描述子
    cv::Mat descriptors_1, descriptors_2;
    orb->compute(img_1, keypoints_1, descriptors_1);
    orb->compute(img_2, keypoints_2, descriptors_2);

    //-- 第三步：对两幅图像中的 BRIEF 描述子进行匹配，使用 Hamming 距离
    std::vector<DMatch> descriptor_pre_matches;
    cv::BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors_1, descriptors_2, descriptor_pre_matches);

    //-- 第四步：筛选匹配点对
    double min_dist = 10000, max_dist = 0;
    // 找出所有匹配之间的最小距离和最大距离
    // 即最相似的和最不相似的亮点之间的距离
    for (int i = 0; i < descriptor_pre_matches.size(); i++)
    {
        double dist = descriptor_pre_matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }
    printf("Max dist: %f\n", max_dist);
    printf("Min dist: %f\n", min_dist);
    
    // 当描述子之间的距离大于两倍的最小距离时，即认为匹配有误
    // 但有时候最小距离非常小，设置一个经验值作为下限

    for (int i = 0; i < descriptor_pre_matches.size(); i++)
    {
        if (descriptor_pre_matches[i].distance <= max(2*min_dist, 30.0))
            matches_optimized.push_back(descriptor_pre_matches[i]);
    }

    //-- 第五步：绘制匹配结果
    cv::Mat show_descriptor_pre_matches;
    cv::Mat show_matches_optimized;
    cv::drawMatches(
        img_1, keypoints_1, img_2, keypoints_2, 
        descriptor_pre_matches, show_descriptor_pre_matches);
    cv::drawMatches(
        img_1, keypoints_1, img_2, keypoints_2, 
        matches_optimized, show_matches_optimized
    );
    cv::imshow("Descriptor pre match", show_descriptor_pre_matches);
    cv::waitKey(0);
    cv::imshow("Matches optimized", show_matches_optimized);
    cv::waitKey(0);
}
```

&emsp;
>主函数
```c++
#include "7.2_features_matching.hpp"

int main(int argc, char** argv)
{
    //-- 读取图像
    cv::Mat img_1 = cv::imread("./data/1.png", IMREAD_COLOR);
    cv::Mat img_2 = cv::imread("./data/2.png", IMREAD_COLOR);
    if (img_1.data == nullptr | img_1.data == nullptr)
        printf("读取图片文件错误，检查路径！");
    
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    std::vector<DMatch> matches_optimized;
    find_feature_matches(
        img_1, img_2, keypoints_1, keypoints_2,
        matches_optimized
    );

    return 0;
}
```


&emsp;
# 后语
1. 当相机为`单目`时，我们只知道 2D 的像素坐标，因而问题是根据两组 2D 点估计运动。该问题用 `对极几何` 来解决。
2. 当相机为 `双目`、`RGB-D` 时，或者我们通过某种方法得到了距离信息，那问题就是根据两组 3D 点估计运动。该问题通常用 `ICP` 来解决。
3. 如果我们有 3D 点和它们在相机的投影位置，也能估计相机的运动。该问题通过 `PnP` 求解。

因此，下面几节的内容，我们就来介绍这三种情形下的相机运动估计。我们将从最基本的 2D-2D 情形出发，看看它如何求解，求解过程又具有哪些麻烦的问题。


