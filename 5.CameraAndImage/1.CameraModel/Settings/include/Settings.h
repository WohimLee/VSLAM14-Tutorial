
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

    private:
        template<typename T>
        T readParameter(cv::FileStorage& fSettings, const std::string& name, bool& found,const bool required = true){
            cv::FileNode node = fSettings[name];
            if(node.empty()){
                if(required){
                    std::cerr << name << " required parameter does not exist, aborting..." << std::endl;
                    exit(-1);
                }
                else{
                    std::cerr << name << " optional parameter does not exist..." << std::endl;
                    found = false;
                    return T();
                }

            }
            else{
                found = true;
                return (T) node;
            }
        }
        
        int sensor_;
    };
};


#endif //ORB_SLAM3_SETTINGS_H
