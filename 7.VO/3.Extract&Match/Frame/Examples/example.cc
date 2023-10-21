
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp> // Mat
#include <opencv2/imgcodecs/imgcodecs.hpp> // imread
#include <opencv2/imgproc/imgproc.hpp>

#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "CameraModels/GeometricCamera.h"

using namespace std;

int main(int argc, char** argv)
{
    // ######### ORBVocabulary #########
    string strVocFile = "/home/nerf/datav/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    ORB_SLAM3::ORBVocabulary* pORBVocabulary = new ORB_SLAM3::ORBVocabulary();
    bool bVocLoad = pORBVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

    // ######### ORBextractor #########
    int nFeatures = 1000;
    int nLevels = 8;
    int fIniThFAST = 20;
    int fMinThFAST = 7;
    float fScaleFactor = 1.2;
    
    ORB_SLAM3::ORBextractor* pIniORBextractor = new ORB_SLAM3::ORBextractor(5*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    
    // ######### LoadImage #########
    std::string img1 = "../imgs/1.png";
    std::string img2 = "../imgs/2.png";

    cv::Mat im;
    im = cv::imread(img1, cv::IMREAD_UNCHANGED);
    cv::Mat imToFeed = im.clone();
    cv::Mat mImGray = imToFeed;

    cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);

    // ######### Camera #########
    ORB_SLAM3::GeometricCamera* pCamera;

    // ######### Frame #########
    cv::Mat mDistCoef;
    float thDepth;

    ORB_SLAM3::Frame curFrame = ORB_SLAM3::Frame(mImGray, pIniORBextractor,pORBVocabulary, pCamera, mDistCoef, thDepth);
    
    return 0;
}

