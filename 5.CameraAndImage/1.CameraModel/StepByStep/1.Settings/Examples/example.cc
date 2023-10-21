
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "Settings.h"

using namespace std;

enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2,
        IMU_MONOCULAR=3,
        IMU_STEREO=4,
        IMU_RGBD=5,
    };

int main(int argc, char** argv)
{

    const string strSettingsFile = "../../../../Common/config/EuRoC.yaml";

    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingsFile << endl;
        exit(-1);
    }

    eSensor sensor = MONOCULAR;

    cv::FileNode node = fsSettings["File.version"];
    if(!node.empty() && node.isString() && node.string() == "1.0"){
        ORB_SLAM3::Settings* settings_ = new ORB_SLAM3::Settings(strSettingsFile, sensor);
    }

    return 0;
}