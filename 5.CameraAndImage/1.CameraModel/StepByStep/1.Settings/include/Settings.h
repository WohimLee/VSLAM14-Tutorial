
#ifndef ORB_SLAM3_SETTINGS_H
#define ORB_SLAM3_SETTINGS_H


#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM3 
{
    class Settings 
    {
    public:
        enum CameraType {
            PinHole = 0,
            Rectified = 1,
            KannalaBrandt = 2
        };

        // Delete default constructor

        Settings() = delete;

        // Constructor from file
        Settings(const std::string &configFile, const int& sensor);

        // Ostream operator overloading to dump settings to the terminal
         
        friend std::ostream &operator<<(std::ostream &output, const Settings &s);
    private:
        int sensor_;
    };
};


#endif //ORB_SLAM3_SETTINGS_H
