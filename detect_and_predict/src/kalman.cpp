#include <kalman.hpp>

kalman::kalman(Eigen::MatrixXd A_,Eigen::MatrixXd B_,
                Eigen::MatrixXd R_,Eigen::MatrixXd Q_,
                Eigen::MatrixXd P_K0,Eigen::MatrixXd X_K0,Eigen::MatrixXd H_)
{
    A = A_;
    B = B_;
    R = R_;
    Q = Q_;
    P_K = P_K0;
    X_K = X_K0;
    H = H_;
}

Eigen::MatrixXd kalman::status()
{
    return X_K;
}
void kalman::update(Eigen::MatrixXd U_K,Eigen::MatrixXd Z_K)
{
    //先根据X_K计算X_K+1的先验
    Eigen::MatrixXd X_K_PLUS_FRONT = predict(U_K);
    //接着计算P_K+1的先验
    Eigen::MatrixXd P_K_PLUS_FRONT = A * P_K * A.transpose() + Q;
    //计算卡尔曼增益K
    Eigen::MatrixXd K = (P_K_PLUS_FRONT * H.transpose()) * 
                    ((H * P_K_PLUS_FRONT * H.transpose() + R).inverse());
    //然后计算X_K+1的后验,即X_K+1的值
    X_K = X_K_PLUS_FRONT + K * (Z_K - H * X_K_PLUS_FRONT);
    //然后更新P_K
    P_K = (Eigen::MatrixXd::Identity(A.rows(),A.cols()) - K * H) * P_K_PLUS_FRONT;
}

Eigen::MatrixXd kalman::predict(Eigen::MatrixXd U_K)
{
    return (A * X_K) + (B * U_K);
}