
#include <string>
#include <iostream>
#include <opencv2/core/core.hpp>

#include "Settings.h"

using namespace std;

int main(int argc, char** argv)
{

    const string strSettingFile = "../../Common/config/EuRoC.yaml";

    cv::FileStorage fsSettings(strSettingFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "Failed to open settings file at: " << strSettingFile << endl;
        exit(-1);
    }

    cv::FileNode node = fsSettings["File.version"];
    if(!node.empty() && node.isString() && node.string() == "1.0"){
        settings_ = new Settings(strSettingsFile,mSensor);
    }

    return 0;
}