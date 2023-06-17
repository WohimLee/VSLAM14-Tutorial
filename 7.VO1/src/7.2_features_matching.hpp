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