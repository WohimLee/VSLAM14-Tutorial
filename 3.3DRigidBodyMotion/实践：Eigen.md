&emsp;
# 3.2 实践：Eigen
- 官方网站：https://eigen.tuxfamily.org/index.php?title=Main_Page
- 官方文档：
- gitlab源码：https://gitlab.com/libeigen/eigen

&emsp;
>头文件
```c++
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
// 计时器
#include <ctime>
// 宏定义，定义矩阵的 “大小”
#define MATRIX_SIZE 50
```
&emsp;
## 1 Eigen::Matrix 声明矩阵
Eigen 以矩阵为基本数据单元。它是一个模板类。它的前三个参数为：
- 数据类型
- 行
- 列

>示例：声明一个 2*3 的 float 矩阵
```c++
Eigen::Matrix<float, 2, 3> matrix_23;
```

同时，Eigen 通过 typedef 提供了许多内置类型，不过底层仍是 `Eigen::Matrix`
- `Vector3d` 列向量：实质上是 `Eigen::Matrix<double, 3, 1>`
- `Matrix3d` 方阵： 实质上是 `Eigen::Matrix<double, 3, 3>`

>示例：声明列向量和方阵
```c++
// 列向量
Eigen::Vector3d vector_31;
Eigen::Vector3d v(1, 0, 0);
// 方阵
Eigen::Matrix3d matrix_33;
```
>示例：声明单位矩阵
```c++
Eigen::Matrix3d matrix = Eigen::Matrix3d::Identity();
```

&emsp;
## 2 不确定大小

如果不确定矩阵大小，可以使用动态大小的矩阵。但这两种不能用 `<<` 输入数据
>示例：
```c++
Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix_dynamic;
// 更简单的
Eigen::MatrixXd matrix_x;
// 这种类型还有很多，我们不一一列举
```

&emsp;
## 3 矩阵的操作
>示例：初始化
```c++
int main()
{
    //（1）随机初始化
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Random();
    //（2）零初始化
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Zero(); 
    return 0;
}
```

>示例：输入输出数据
```c++
matrix_23 << 1, 2, 3, 4, 5, 6;
// 输出
cout << matrix_23 << endl;
```

>示例：用 () 访问数据
```c++
int main()
{
    Eigen::Matrix<float, 2, 3> matrix;
    matrix << 1,2,3,4,5,6;
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 3; j++)
            cout << matrix(i, j) << " ";
        cout << endl;       
    }
    return 0;
}
```

&emsp;
## 4 矩阵的乘法

>示例：矩阵和向量相乘
```c++
// 矩阵和向量相乘（实际上仍是矩阵和矩阵）
// 但是在这里你不能混合两种不同数据类型的矩阵，像这样是错的
Eigen::Matrix<double, 2, 1> result_wrong_type = matrix_23 * v_3d;
// 应该显式转换
int main()
{

    Eigen::Matrix<float, 2, 3> matrix;
    Eigen::Vector3d vector;
    matrix << 1,2,3,4,5,6;
    vector << 3, 2, 1;
    // 通过 .cast<类型>() 转换为一致的数据类型
    Eigen::Matrix<double, 2, 1> result = matrix.cast<double>() * vector;
    std::cout << result << std::endl;
    return 0;
}
```


>示例：注意维度问题
```c++
// 同样你不能搞错矩阵的维度
// 试着运行下面的语句，看看会报什么错
Eigen::Matrix<double, 2, 3> result_wrong_dimension = matrix.cast<double>() * vector;
```

&emsp;
## 5 其它运算
>示例：转置、求和、迹、数乘、逆、行列式
```c++
int main()
{
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Random();
    cout << matrix_33 << endl << endl;
    // （1）转置
    cout << matrix_33.transpose() << endl; 
    // （2）各元素和
    cout << matrix_33.sum() << endl; 
    // （3）迹
    cout << matrix_33.trace() << endl; 
    // （4）数乘
    cout << 10*matrix_33 << endl; 
    // （5）逆
    cout << matrix_33.inverse() << endl; 
    // （6）行列式
    cout << matrix_33.determinant() << endl; 
    return 0;
}
```

>示例：求特征值与特征向量
```c++
int main()
{
    Eigen::Matrix3d matrix_33 = Eigen::Matrix3d::Random();

    // 实对称矩阵可以保证对角化成功
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>eigen_solver(matrix_33.transpose()*matrix_33);
    // 特征值
    cout << "Eigen values = " << eigen_solver.eigenvalues() << endl;
    // 特征向量
    cout << "Eigen vectors = " << eigen_solver.eigenvectors() << endl;
    return 0;
}
```

>示例：解方程
```c++
int main()
{
    // 我们求解 matrix_NN * x = v_Nd 这个方程
    // N 的大小在前边的宏里定义，矩阵由随机数生成
    // 直接求逆自然是最直接的，但是求逆运算量大
    Eigen::Matrix< double, MATRIX_SIZE, MATRIX_SIZE > matrix_NN;
    matrix_NN = Eigen::MatrixXd::Random(MATRIX_SIZE, MATRIX_SIZE);
    Eigen::Matrix< double, MATRIX_SIZE, 1> v_Nd;
    v_Nd = Eigen::MatrixXd::Random(MATRIX_SIZE,1);

    clock_t time_stt = clock(); // 计时
    // 直接求逆
    Eigen::Matrix<double,MATRIX_SIZE, 1> x = matrix_NN.inverse()*v_Nd;
    cout << "time use in normal invers is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms" << endl;

    // 通常用矩阵分解来求，例如 QR 分解，速度会快很多
    time_stt = clock();
    x = matrix_NN.colPivHouseholderQr().solve(v_Nd);
    cout << "time use in Qr compsition is " << 1000* (clock() - time_stt)/(double)CLOCKS_PER_SEC << "ms" << endl;
    return 0;
}
```


