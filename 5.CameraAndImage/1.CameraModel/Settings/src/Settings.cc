
#include <iostream>
#include "Settings.h"

using namespace std;

namespace ORB_SLAM3
{
    
    Settings::Settings(const std::string &configFile, const int& sensor)
    {
        sensor_ = sensor;

        //Open settings file
        cv::FileStorage fSettings(configFile, cv::FileStorage::READ);
        if (!fSettings.isOpened()) {
            cerr << "[ERROR]: could not open configuration file at: " << configFile << endl;
            cerr << "Aborting..." << endl;

            exit(-1);
        }
        else{
            cout << "Loading settings from " << configFile << endl;
        }
        // //Read first camera
        // readCamera1(fSettings);
        // cout << "\t-Loaded camera 1" << endl;

        // //Read image info
        // readImageInfo(fSettings);
        // cout << "\t-Loaded image info" << endl;

        // readORB(fSettings);
        // cout << "\t-Loaded ORB settings" << endl;

        // cout << "----------------------------------" << endl;
    }


}