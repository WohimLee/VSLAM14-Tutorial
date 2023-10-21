#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace ORB_SLAM3
{
class ORBextractor
{
public:
    std::vector<cv::Mat> mvImagePyramid;



    void ComputePyramid(cv::Mat image);

    int nlevels;

    std::vector<float> mvScaleFactor;
    std::vector<float> mvInvScaleFactor;    
    std::vector<float> mvLevelSigma2;
    std::vector<float> mvInvLevelSigma2;
};

}

#endif // ORBEXTRACTOR_H