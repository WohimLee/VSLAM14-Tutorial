
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp> // Mat
#include <opencv2/imgcodecs/imgcodecs.hpp> // imread
#include <opencv2/imgproc/imgproc.hpp>



using namespace std;

int main(int argc, char** argv)
{
    std::string img1 = "../imgs/1.png";
    std::string img2 = "../imgs/2.png";

    cv::Mat im;
    im = cv::imread(img1, cv::IMREAD_UNCHANGED);
    cv::Mat imToFeed = im.clone();
    cv::Mat mImGray = imToFeed;

    cv::cvtColor(mImGray, mImGray, cv::COLOR_RGB2GRAY);

    
    return 0;
}

