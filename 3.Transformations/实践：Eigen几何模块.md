&emsp;
# 实践：Eigen 几何模块

现在，我们来实际演练一下前面讲到的各种旋转表达方式。我们将在 Eigen 中使用四元数、欧拉角和旋转矩阵，演示它们之间的变换方式。我们还会给出一个可视化程序，帮助读者理解这几个变换的关系。

>头文件
```c++
#include <iostream>
#include <cmath>
using namespace std;

#include <Eigen/Core>
// Eigen 几何模块
#include <Eigen/Geometry>
```

Eigen 中对各种形式的表达方式总结如下。请注意每种类型都有`单精度`和`双精度`两种数据类型，而且和之前一样，不能由编译器自动转换。下面以双精度为例，你可以把最后的 `d` 改成 `f`，即得到单精度的数据结构。
- 旋转矩阵（3 × 3）：Eigen::Matrix3d。
- 旋转向量（3 × 1）：Eigen::AngleAxisd
- 欧拉角（3 × 1）：Eigen::Vector3d
- 四元数（4 × 1）：Eigen::Quaterniond
- 欧氏变换矩阵（4 × 4）：Eigen::Isometry3d
- 仿射变换（4 × 4）：Eigen::Affine3d
- 射影变换（4 × 4）：Eigen::Projective3d

&emsp;

Eigen/Geometry 模块提供了各种旋转和平移的表示

## 1 声明与定义
>旋转矩阵 $\pmb{R}$
- 3D 旋转矩阵直接使用 Matrix3d 或 Matrix3f
```c++
// 用单位矩阵作初始化
Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();
```

&emsp;
>旋转向量 $θ\pmb{n}$
- 使用 AngleAxis, 它底层不直接是 Matrix ，但运算可以当作矩阵（因为重载了运算符）
```c++
// 沿 Z 轴旋转 45 度
Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); 
```


&emsp;
>欧式变换矩阵 $\pmb{T}$
- 声明欧氏变换矩阵 $\pmb{T}$ 使用 Eigen::Isometry
```c++
int main(int argc, char** argv)
{
    // 声明旋转向量（角轴，Angle Axis）
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); 

    // 虽然称为 3d ，实质上是 4＊4 的矩阵 
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); 
    // （1）设置旋转向量 rotation_vector 
    T.rotate(rotation_vector); 
    // （2）设置平移向量  (1,3,4)
    T.pretranslate(Eigen::Vector3d(1,3,4)); 
    cout << "Transform matrix = \n" << T.matrix() <<endl;
}
```

>四元数
```c++
Eigen::Quaterniond q;
```


&emsp;
## 2 不同表示方式的转换

>$θ\pmb{n}$ 转 $\pmb{R}$
- 用 matrix() 转换成矩阵，也可以用 toRotationMatrix()
```c++
int main(int argc, char** argv)
{
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1));
    cout.precision(3);
    // 用 matrix() 转换成矩阵
    cout << "rotation matrix = \n" << rotation_vector.matrix() << endl;
    // 也可以用 toRotationMatrix()
    cout << "rotation matrix = \n" << rotation_vector.toRotationMatrix() << endl;
    return 0;
}
```

&emsp;
>$\pmb{R}$ 转 欧拉角$[r, p, y]^T$
- 用 eulerAngles() 可以将旋转矩阵直接转换成欧拉角
```c++
int main(int argc, char** argv)
{
    // 声明向量 v(1, 0, 0)^T
    Eigen::Vector3d v(1, 0, 0);
    // 声明旋转向量（角轴，Angle Axis）
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); 
    // Angel Axis 转 Rotation Matrix
    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
    // Rotation Matrix 转 Euler Angles
    // 2, 1, 0 分别对应 Z Y X，即 yaw pitch roll
    Eigen::Vector3d euler_angles = rotation_matrix.eulerAngles(2, 1, 0);
    cout << "yaw pitch roll = " << euler_angles.transpose() << endl;
    return 0;
}
```
&emsp;
>转四元数
- 可以直接把 $θ\pmb{n}$ （角轴/轴角，Angle Axis）赋值给四元数，反之亦然
```c++
int main(int argc, char** argv)
{
    // 声明向量 v(1, 0, 0)^T
    Eigen::Vector3d v(1, 0, 0);
    // 声明旋转向量（角轴，Angle Axis）
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); 
    
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);
    // 请注意 coeffs 的顺序是 (x,y,z,w), w 为实部，前三者为虚部
    cout << "quaternion = \n" << q.coeffs() << endl; 
}
```
- 也可以把 $\pmb{R}$ （旋转矩阵）赋给它
```c++
int main(int argc, char** argv)
{
    // 声明向量 v(1, 0, 0)^T
    Eigen::Vector3d v(1, 0, 0);
    // 声明旋转向量（角轴，Angle Axis）
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); 
    // Angel Axis 转 Rotation Matrix
    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
    
    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix);
    cout<<"quaternion = \n"<<q.coeffs() <<endl;
}
```

&emsp;
## 3 旋转的表示


>旋转向量 $θ\pmb{n}$
- 用旋转向量 $θ\pmb{n}$ （角轴/轴角，Angle Axis）表示旋转
```c++
int main(int argc, char** argv)
{
    // 声明旋转向量（角轴，Angle Axis）
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); 
    // 声明向量 v(1, 0, 0)^T
    Eigen::Vector3d v(1, 0, 0);
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;
    return 0;
}
```
>旋转矩阵 $\pmb{R}$
- 用旋转矩阵 $\pmb{R}$ 表示旋转
```c++
int main(int argc, char** argv)
{
    // 声明旋转向量（角轴，Angle Axis）
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); 
    // 声明向量 v(1, 0, 0)^T
    Eigen::Vector3d v(1, 0, 0);
    
    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();
    Eigen::Vector3d v_rotated = rotation_vector * v;
    cout << "(1, 0, 0) after rotation = " << v_rotated.transpose() << endl;
    return 0;
}
```

>变换矩阵 $T$
- 用变换矩阵进行旋转
```c++
int main(int argc, char** argv)
{
    // 声明向量 v(1, 0, 0)^T
    Eigen::Vector3d v(1, 0, 0);
    // 声明旋转向量（角轴，Angle Axis）
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); 
    
    // 虽然称为 3d ，实质上是 4＊4 的矩阵 
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); 
    // 设置旋转向量 rotation_vector 
    T.rotate(rotation_vector); 
    // 设置平移向量  (1,3,4)
    T.pretranslate(Eigen::Vector3d(1,3,4)); 
    cout << "Transform matrix = \n" << T.matrix() <<endl;
    Eigen::Vector3d v_transformed = T*v; // 相当于 R*v+t
    cout<<"v tranformed = "<<v_transformed.transpose()<<endl;
}
```

&emsp;
>四元数旋转
- 使用四元数旋转一个向量，使用重载的乘法即可，注意数学上是 $\pmb{qvq}^{-1}$
```c++
int main(int argc, char** argv)
{
    // 声明向量 v(1, 0, 0)^T
    Eigen::Vector3d v(1, 0, 0);
    // 声明旋转向量（角轴，Angle Axis）
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1)); 
    // Angel Axis 转 Rotation Matrix
    Eigen::Matrix3d rotation_matrix = rotation_vector.toRotationMatrix();

    Eigen::Quaterniond q = Eigen::Quaterniond(rotation_matrix);
    
    Eigen::Vector3d v_rotated = q*v; 
    cout << "(1,0,0) after rotation = " << v_rotated.transpose()<<endl;
}
```

&emsp;
## 4 仿射变换和射影变换
>仿射变换和射影变换
- 对于仿射和射影变换，使用 Eigen::Affine3d 和 Eigen::Projective3d 即可，略





