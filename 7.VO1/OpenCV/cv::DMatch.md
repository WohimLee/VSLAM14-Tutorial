&emsp;
# cv::DMatch
- https://docs.opencv.org/4.x/d4/de0/classcv_1_1DMatch.html

## Public Member Functions
- DMatch ()
- DMatch (int _queryIdx, int _trainIdx, float _distance)
- DMatch (int _queryIdx, int _trainIdx, int _imgIdx, float _distance)
 - bool 	operator< (const DMatch &m) const
 
## Public Attributes
- float distance
- int imgIdx
    - train image index More...
- int queryIdx
    - query descriptor index More...
- int trainIdx
    - train descriptor index More...
 
## Detailed Description
- Class for matching keypoint descriptors.
- query descriptor index, train descriptor index, train image index, and distance between descriptors.