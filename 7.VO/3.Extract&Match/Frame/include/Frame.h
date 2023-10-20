#ifndef FRAME_H
#define FRAME_H

#include <opencv2/core/core.hpp>
#include "ORBVocabulary.h"

namespace ORB_SLAM3
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;
class ConstraintPoseImu;
class GeometricCamera;
class ORBextractor;

class Frame
{
public:
    Frame();

    // Copy constructor.
    Frame(const Frame &frame);

    // Constructor for Monocular cameras.
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, GeometricCamera* pCamera, cv::Mat &distCoef, const float &bf, const float &thDepth, Frame* pPrevF = static_cast<Frame*>(NULL));

    // Destructor
    // ~Frame();

    // Extract ORB on the image. 0 for left image and 1 for right image.
    void ExtractORB(int flag, const cv::Mat &im, const int x0, const int x1);
}; //
} //

#endif // FRAME_H

