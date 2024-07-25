#include <Eigen/Dense>
#include <iostream>


class kalman
{
private:
    Eigen::MatrixXd A;    //无控制下的转移矩阵
    Eigen::MatrixXd B;    //控制转移矩阵
    Eigen::MatrixXd X_K;  //在k时间X的预测值
    Eigen::MatrixXd H;    //少->多方程
    Eigen::MatrixXd R;    //过程噪声协方差矩阵
    Eigen::MatrixXd Q;    //测量噪声协方差矩阵
    Eigen::MatrixXd P_K;  //预测误差协方差矩阵
public:
    kalman(Eigen::MatrixXd A_,Eigen::MatrixXd B_,Eigen::MatrixXd R_,
            Eigen::MatrixXd Q_,Eigen::MatrixXd P_K0,Eigen::MatrixXd X_K0,Eigen::MatrixXd H_);
    Eigen::MatrixXd update(Eigen::MatrixXd U_K,Eigen::MatrixXd Z_K);

};
