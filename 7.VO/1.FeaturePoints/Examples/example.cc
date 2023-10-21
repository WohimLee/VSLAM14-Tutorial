
#include <iostream>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include "ORBextractor.h"

using namespace std;


int main(int argc, char** argv)
{
    // ######### LoadImage #########
    std::string img1 = "../../Common/data/1.png";
    std::string img2 = "../../Common/data/2.png";

    cv::Mat im;
    im = cv::imread(img1, cv::IMREAD_UNCHANGED);
    cv::Mat imToFeed = im.clone();
    cv::Mat mImGray = imToFeed;

    cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);

    ORB_SLAM3::ORBextractor extractor;
    extractor.ComputePyramid(mImGray);

    cout << extractor.nlevels << endl;
    

    return 0;
}