#include <iostream>
// Eigen 部分
#include <Eigen/Core>
// 稠密矩阵的代数运算（逆，特征值等）
#include <Eigen/Dense>
// 计时器
#include <ctime>
#define MATRIX_SIZE 50
using namespace std;

int main()
{
    // 解方程
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


