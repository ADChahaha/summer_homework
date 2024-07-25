#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <utility>
#include "kalman.hpp"
#include <string>

int main(int argc,char** argv)
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(1,1);
    Eigen::MatrixXd B = Eigen::MatrixXd::Identity(1,1);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(1,1) * 1;       
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(1,1) * 0.5;
    Eigen::MatrixXd P_K0 = Eigen::MatrixXd::Identity(1,1) * 10000;
    Eigen::MatrixXd X_K0 = Eigen::MatrixXd::Zero(1,1);
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(1,1);
    kalman filter(A,B,R,Q,P_K0,X_K0,H);
    std::string path = "data/";
    std::string param_path = argv[2];
    path += argv[1];
    std::ifstream ifs(path);
    std::ofstream ofs(param_path);
    std::string line;
    double x,y;
    char c;
    std::getline(ifs,line);
    while(std::getline(ifs,line))
    {
        std::istringstream lineStream(line);
        lineStream >> x >> c >> y;
        Eigen::MatrixXd U = Eigen::MatrixXd::Zero(1,1);
        Eigen::MatrixXd Z = Eigen::MatrixXd::Zero(1,1);
        U(0,0) = 0.0;
        Z(0,0) = y;
        //std::cout << "x: " << x << " y: " << y << std::endl;
        auto matrix = filter.update(U,Z);
        std::cout << matrix << std::endl;
        ofs << x << ' ' << matrix(0,0) << std::endl;
    }
    return 0;
}
