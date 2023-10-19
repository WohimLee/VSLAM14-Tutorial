#ifndef POSE_ESTIMATION_2D2D_H
#define POSE_ESTIMATION_2D2D_H


#include <vector>
#include <opencv2/core/core.hpp>


void find_feature_matches(
    const cv::Mat &img_1, const cv::Mat &img_2,
    std::vector<cv::KeyPoint> &keypoints_1,
    std::vector<cv::KeyPoint> &keypoints_2,
    std::vector<cv::DMatch> &matches);

void pose_estimation_2d2d(
    std::vector<cv::KeyPoint> keypoints_1,
    std::vector<cv::KeyPoint> keypoints_2,
    std::vector<cv::DMatch> matches,
    cv::Mat &R, cv::Mat &t);

// 像素坐标转相机归一化坐标
cv::Point2d pixel2cam(const cv::Point2d &p, const cv::Mat &K);

#endif // POSE_ESTIMATION_2D2D_H