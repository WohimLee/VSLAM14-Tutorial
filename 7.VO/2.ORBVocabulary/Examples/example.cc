
#include <iostream>
#include <string>
#include "ORBVocabulary.h"


using namespace std;

int main(int argc, char** argv)
{
    string strVocFile = "/home/nerf/datav/SLAM/ORB_SLAM3/Vocabulary/ORBvoc.txt";
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    ORB_SLAM3::ORBVocabulary* pVocabulary = new ORB_SLAM3::ORBVocabulary();
    bool bVocLoad = pVocabulary->loadFromTextFile(strVocFile);
        if(!bVocLoad)
        {
            cerr << "Wrong path to vocabulary. " << endl;
            cerr << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;
    return 0;
}