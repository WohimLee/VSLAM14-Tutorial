&emsp;
# recoverPose [3/4]
- This is an overloaded member function, provided for convenience. It differs from the above function only in what argument(s) it accepts.

>header
```c++
#include <opencv2/calib3d.hpp>
```

## API
```c++
int cv::recoverPose	(	
    InputArray 	E,
    InputArray 	points1,
    InputArray 	points2,
    OutputArray 	R,
    OutputArray 	t,
    double 	focal = 1.0,
    Point2d pp = Point2d(0, 0),
    InputOutputArray 	mask = noArray() 
)		
```

## Parameters
- `E`：The input essential matrix.
- `points1`：Array of N 2D points from the first image. The point coordinates should be floating-point (single or double precision).
- `points2`：Array of the second image points of the same size and format as points1 .
- `R`：Output rotation matrix. Together with the translation vector, this matrix makes up a tuple that performs a change of basis from the first camera's coordinate system to the second camera's coordinate system. Note that, in general, t can not be used for this tuple, see the parameter description below.
- `t`：Output translation vector. This vector is obtained by decomposeEssentialMat and therefore is only known up to scale, i.e. t is the direction of the translation vector and has unit length.
- `focal`：Focal length of the camera. Note that this function assumes that points1 and points2 are feature points from cameras with same focal length and principal point.
- `pp`：principal point of the camera.
- `mask`：Input/output mask for inliers in points1 and points2. If it is not empty, then it marks inliers in points1 and points2 for the given essential matrix E. Only these inliers will be used to recover pose. In the output mask only inliers which pass the chirality check.


This function differs from the one above that it computes camera intrinsic matrix from focal length and principal point:

$$\pmb{A} = \begin{bmatrix}
f & 0 & x_pp \\ 0&f&y_pp \\ 0&0&1
\end{bmatrix}$$