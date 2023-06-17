&emsp;
# findFundamentalMat
- 一共有4种重载方式

Calculates a fundamental matrix from the corresponding points in two images.


## API
>header
```c++
#include <opencv2/calib3d.hpp>
```

>C++
```c++
Mat cv::findFundamentalMat(	
    InputArray 	points1,
    InputArray 	points2,
    int 	method,
    double 	ransacReprojThreshold,
    double 	confidence,
    int 	maxIters,
    OutputArray 	mask = noArray() 
)
```		




## Parameters
- `points1`：Array of N points from the first image. The point coordinates should be floating-point (single or double precision).

- `points2`：Array of the second image points of the same size and format as points1 .
- `method`：Method for computing a fundamental matrix.
    - `FM_7POINT` for a 7-point algorithm. N=7
    
    - `FM_8POINT` for an 8-point algorithm. N≥8
    - `FM_RANSAC` for the RANSAC algorithm. N≥8
    - `FM_LMEDS` for the LMedS algorithm. N≥8
- `ransacReprojThreshold`：Parameter used only for RANSAC. It is the maximum distance from a point to an epipolar line in pixels, beyond which the point is considered an outlier and is not used for computing the final fundamental matrix. It can be set to something like 1-3, depending on the accuracy of the point localization, image resolution, and the image noise.
- `confidence`：Parameter used for the RANSAC and LMedS methods only. It specifies a desirable level of confidence (probability) that the estimated matrix is correct.
[out]	mask	optional output mask
maxIters	The maximum number of robust method iterations.


The epipolar geometry is described by the following equation:

$$[p2;1]^TF[p1;1]=0$$

- where `F` is a fundamental matrix
- `p1` and `p2` are corresponding points in the first and the second images, respectively.

The function calculates the fundamental matrix using one of four methods listed above and returns the found fundamental matrix. Normally just one matrix is found. But in case of the 7-point algorithm, the function may return up to 3 solutions ( 9×3 matrix that stores all 3 matrices sequentially).

&emsp;
## Example. 
- The calculated fundamental matrix may be passed further to computeCorrespondEpilines that finds the epipolar lines corresponding to the specified points. It can also be passed to stereoRectifyUncalibrated to compute the rectification transformation.

>Estimation of fundamental matrix using the RANSAC algorithm
```c++
int point_count = 100;
vector<Point2f> points1(point_count);
vector<Point2f> points2(point_count);
// initialize the points here ...
for( int i = 0; i < point_count; i++ )
{
    points1[i] = ...;
    points2[i] = ...;
}
Mat fundamental_matrix =
 findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99);
```