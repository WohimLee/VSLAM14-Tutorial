&emsp;
# cv::KeyPoint
- https://docs.opencv.org/4.x/d2/d29/classcv_1_1KeyPoint.html

## Public Member Functions
```c++
// the default constructor More... 
KeyPoint ()

KeyPoint (
    Point2f pt, float size, float angle=-1, 
    float response=0, int octave=0, int class_id=-1)

KeyPoint (
    float x, float y, float size, 
    float angle=-1, float response=0, int octave=0, 
    int class_id=-1)

size_t 	hash () const
```

## Static Public Member Functions
```c++
static void convert(
    const std::vector< KeyPoint > &keypoints, 
    std::vector< Point2f > &points2f, 
    const std::vector< int > &keypointIndexes=std::vector< int >())

static void convert(
    const std::vector< Point2f > &points2f, 
    std::vector< KeyPoint > &keypoints, 
    float size=1, 
    float response=1, 
    int octave=0, 
    int class_id=-1)

static float overlap(
    const KeyPoint &kp1, 
    const KeyPoint &kp2)
```
## Public Attributes
```c++
float angle

// object class (if the keypoints need to be clustered by an object they belong to) More...
int class_id

 // octave (pyramid layer) from which the keypoint has been extracted More...
int octave

// coordinates of the keypoints More...
Point2f pt

// the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling More...
float response

// diameter of the meaningful keypoint neighborhood More...
float size
```


## Detailed Description
Data structure for salient point detectors.

The class instance stores a keypoint, i.e. a point feature found by one of many available keypoint detectors, such as Harris corner detector, FAST, StarDetector, SURF, SIFT etc.

The keypoint is characterized by the 2D position, scale (proportional to the diameter of the neighborhood that needs to be taken into account), orientation and some other parameters. The keypoint neighborhood is then analyzed by another algorithm that builds a descriptor (usually represented as a feature vector). The keypoints representing the same object in different images can then be matched using KDTree or another method.

