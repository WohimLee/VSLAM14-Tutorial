&emsp;
# 7.8.2 使用 BA 优化

下面，我们来演示如何进行 $Bundle Adjustment$。我们将使用前一步的估计值作为初始值。优化可以使用前面讲的 $Ceres$ 或 $g2o$ 库实现，这里采用 $g2o$ 作为例子。

$g2o$ 的基本知识在第六讲中已经介绍过了。在使用 $g2o$ 之前，我们要把问题建模成一
个最小二乘的图优化问题，如图 7-13 所示。在这个图优化中，节点和边的选择为：

<div align="center">
    <image src="./imgs/7.8-1.png" width = 500>
</div>
&emsp;

1. 节点：第二个相机的位姿节点 $ξ ∈ \mathfrak{se}(3)$，以及所有特征点的空间位置 $\pmb{P} ∈ \mathbb{R}^3$。
2. 边：每个 3D 点在第二个相机中的投影，以观测方程来描述：
$$z_j = h(ξ, \pmb{P}_j )$$

由于第一个相机位姿固定为零，我们没有把它写到优化变量里，但在习题中，我希望你能够把第一个相机的位姿与观测也考虑进来。现在我们根据一组 $3D$ 点和第二个图像中的 $2D$ 投影，估计第二个相机的位姿。所以我们把第一个相机画成虚线，表明我们不希望考虑它。

&emsp;
## 1 g2o 提供的节点、边
$g2o$ 提供了许多关于 $BA$ 的节点和边，我们不必自己从头实现所有的计算。在 g2o/types/sba/types_six_dof_expmap.h 中则提供了李代数表达的节点和边。请读者打开这个文件，找到 VertexSE3Expmap（李代数位姿）、VertexSBAPointXYZ（空间点位置）和 EdgeProjectXYZ2UV（投影方程边）这三个类。我们来简单看一下它们的类定义，例如 

>VertexSE3Expmap
```c++
class G2O_TYPES_SBA_API VertexSE3Expmap : public BaseVertex<6, SE3Quat>{
public: 3 EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VertexSE3Expmap();
    bool read(std::istream& is);
    bool write(std::ostream& os) const;

    virtual void setToOriginImpl() {
    _estimate = SE3Quat();
    }

    virtual void oplusImpl(const double* update_) {
        Eigen::Map<const Vector6d> update(update_);
        setEstimate( SE3Quat::exp(update)*estimate());
    }
};
```

请注意它的模板参数。第一个参数 $6$ 表示它内部存储的优化变量维度，可以看到这是一个 $6$ 维的李代数。第二参数是优化变量的类型，这里使用了 $g2o$ 定义的相机位姿：`SE3Quat`。

这个类内部使用了四元数加位移向量来存储位姿，但同时也支持李代数上的运算，例如对数映射（log 函数）和李代数上增量（update 函数）等操作。我们可以对照它的实现代码，

看看 g2o 对李代数是如何操作的：

>
```c++
class G2O_TYPES_SBA_API VertexSBAPointXYZ : public BaseVertex<3, Vector3D>
{   ......

};
class G2O_TYPES_SBA_API EdgeProjectXYZ2UV : public BaseBinaryEdge<2, Vector2D, VertexSBAPointXYZ, VertexSE3Expmap>
{   ......
    void computeError() {
        const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
        const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
        const CameraParameters * cam
            = static_cast<const CameraParameters *>(parameter(0));
        Vector2D obs(_measurement);
        _error = obs-cam->cam_map(v1->estimate().map(v2->estimate()));
    }
};
```
我就不把整个类定义都搬过来了。从模板参数可以看到，空间点位置类的维度为 3，类型是 $Eigen$ 的 `Vector3D`。另一方面，边 `EdgeProjectXYZ2UV` 连接了两个前面说的两个顶点，它的观测值为 $2$ 维，由 `Vector2D` 表示，实际上就是空间点的像素坐标。它的误差计算函数表达了投影方程的误差计算方法，也就是我们前面提到的 $z - h(\pmb{ξ}, \pmb{P})$ 的方式。

现在，进一步观察 `EdgeProjectXYZ2UV` 的 `linearizeOplus` 函数的实现。这里用到了我们前面推导的雅可比矩阵：

```c++
void EdgeProjectXYZ2UV::linearizeOplus() {
    VertexSE3Expmap * vj = static_cast<VertexSE3Expmap *>(_vertices[1]);
    SE3Quat T(vj->estimate());
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Vector3D xyz = vi->estimate();
    Vector3D xyz_trans = T.map(xyz);
    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z*z;

    const CameraParameters * cam = static_cast<const CameraParameters *>(parameter(0));

    Matrix<double,2,3,Eigen::ColMajor> tmp;
    tmp(0,0) = cam->focal_length;
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*cam->focal_length;
    tmp(1,0) = 0;
    tmp(1,1) = cam->focal_length;
    tmp(1,2) = -y/z*cam->focal_length;

    _jacobianOplusXi = -1./z * tmp * T.rotation().toRotationMatrix();

    _jacobianOplusXj(0,0) = x*y/z_2 *cam->focal_length;
    _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) *cam->focal_length;
    _jacobianOplusXj(0,2) = y/z *cam->focal_length;
    _jacobianOplusXj(0,3) = -1./z *cam->focal_length;
    _jacobianOplusXj(0,4) = 0;
    _jacobianOplusXj(0,5) = x/z_2 *cam->focal_length;

    _jacobianOplusXj(1,0) = (1+y*y/z_2) *cam->focal_length;
    _jacobianOplusXj(1,1) = -x*y/z_2 *cam->focal_length;
    _jacobianOplusXj(1,2) = -x/z *cam->focal_length;
    _jacobianOplusXj(1,3) = 0;
    _jacobianOplusXj(1,4) = -1./z *cam->focal_length;
    _jacobianOplusXj(1,5) = y/z_2 *cam->focal_length;
}
```



仔细研究此段代码，我们会发现它与式$（7.45）$和$（7.47）$是一致的。成员变量 `_-jacobianOplusXi` 是误差到空间点的导数，`_jacobianOplusXj` 是误差到相机位姿的导数，以李代数的左乘扰动表达。稍有差别的是，$g2o$ 的相机里用 $f$ 统一描述 $f_x, f_y$，并且李代数定义顺序不同（$g2o$ 是旋转在前，平移在后；我们是平移在前，旋转在后），所以矩阵前三列和后三列与我们的定义是颠倒的。此外都是一致的。

值得一提的是，我们亦可自己实现相机位姿节点，并使用 `Sophus::SE3` 来表达位姿，提供类似的求导过程。然而，既然 $g2o$ 已经提供了这样的类，在没有额外要求的情况下，自己重新实现就没有必要了。

&emsp;
## 2 代码

现在，我们在上一个 $PnP$ 例程的基础上，加上 $g2o$ 提供的 Bundle Adjustment。

迭代 $11$ 轮后，$LM$ 发现优化目标函数接近不变，于是停止了优化。我们输出了最后得到位姿变换矩阵 $\pmb{T}$，对比之前直接做 $PnP$ 的结果，大约在小数点后第三位发生了一些变化。这主要是由于我们同时优化了特征点和相机位姿导致的。

Bundle Adjustment 是一种通用的做法。它可以不限于两个图像。我们完全可以放入多个图像匹配到的位姿和空间点进行迭代优化，甚至可以把整个 SLAM 过程放进来。那种做法规模较大，主要在后端使用，我们会在第十章重新遇到这个问题。在前端，我们通常考虑局部相机位姿和特征点的小型 Bundle Adjustment 问题，希望实时对它进行求解和优化。

&emsp;
>7.8_pose_estimation_3d2d.cpp
```c++
void bundleAdjustment (
    const vector<Point3f> points_3d,
    const vector<Point2f> points_2d,
    const Mat& K,
    Mat& R, Mat& t
);
```

&emsp;
>7.8_pose_estimation_3d2d.cpp
```c++
void bundleAdjustment (
    const vector< Point3f > points_3d,
    const vector< Point2f > points_2d,
    const Mat& K,
    Mat& R, Mat& t )
{
    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;  // pose 维度为 6, landmark 维度为 3
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block( 
        std::unique_ptr<Block::LinearSolverType>(linearSolver));     // 矩阵块求解器std::unique_ptr<Block>(solver_ptr)
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<Block>(solver_ptr) );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // vertex
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
    Eigen::Matrix3d R_mat;
    R_mat <<
          R.at<double> ( 0,0 ), R.at<double> ( 0,1 ), R.at<double> ( 0,2 ),
               R.at<double> ( 1,0 ), R.at<double> ( 1,1 ), R.at<double> ( 1,2 ),
               R.at<double> ( 2,0 ), R.at<double> ( 2,1 ), R.at<double> ( 2,2 );
    pose->setId ( 0 );
    pose->setEstimate ( g2o::SE3Quat (
                            R_mat,
                            Eigen::Vector3d ( t.at<double> ( 0,0 ), t.at<double> ( 1,0 ), t.at<double> ( 2,0 ) )
                        ) );
    optimizer.addVertex ( pose );

    int index = 1;
    for ( const Point3f p:points_3d )   // landmarks
    {
        g2o::VertexPointXYZ* point = new g2o::VertexPointXYZ();
        point->setId ( index++ );
        point->setEstimate ( Eigen::Vector3d ( p.x, p.y, p.z ) );
        point->setMarginalized ( true ); // g2o 中必须设置 marg 参见第十讲内容
        optimizer.addVertex ( point );
    }

    // parameter: camera intrinsics
    g2o::CameraParameters* camera = new g2o::CameraParameters (
        K.at<double> ( 0,0 ), Eigen::Vector2d ( K.at<double> ( 0,2 ), K.at<double> ( 1,2 ) ), 0
    );
    camera->setId ( 0 );
    optimizer.addParameter ( camera );

    // edges
    index = 1;
    for ( const Point2f p:points_2d )
    {
        g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
        edge->setId ( index );
        edge->setVertex ( 0, dynamic_cast<g2o::VertexPointXYZ*> ( optimizer.vertex ( index ) ) );
        edge->setVertex ( 1, pose );
        edge->setMeasurement ( Eigen::Vector2d ( p.x, p.y ) );
        edge->setParameterId ( 0,0 );
        edge->setInformation ( Eigen::Matrix2d::Identity() );
        optimizer.addEdge ( edge );
        index++;
    }

    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    optimizer.setVerbose ( true );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> ( t2-t1 );
    cout<<"optimization costs time: "<<time_used.count() <<" seconds."<<endl;

    cout<<endl<<"after optimization:"<<endl;
    cout<<"T="<<endl<<Eigen::Isometry3d ( pose->estimate() ).matrix() <<endl;
}
```