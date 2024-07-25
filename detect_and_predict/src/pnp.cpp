#include "pnp.hpp"


pnp_solver::pnp_solver(const cv::Mat& camera,const cv::Mat& dist)
{
    cameraMatrix = camera;
    distCoeffs = dist;
}

void pnp_solver::updateParam(const cv::Mat& camera,const cv::Mat& dist)
{
    cameraMatrix = camera;
    distCoeffs = dist;
}

void pnp_solver::solve(const Armor& armor,cv::Mat& rvec,cv::Mat& tvec)
{
    std::vector<cv::Point2f> four_points;
    four_points.push_back(armor.left_light.bottom);
    four_points.push_back(armor.left_light.top);
    four_points.push_back(armor.right_light.top);
    four_points.push_back(armor.right_light.bottom);
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<    2102.080562187802,
                                                        0,
                                                        689.2057889332623,
                                                        0,
                                                        2094.0179120166754,
                                                        496.6622802275393,
                                                        0,
                                                        0,
                                                        1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1,5) << -0.06478109387525666,
                                                    0.39036067923005396,
                                                    -0.0042514793151166306,
                                                    0.008306749648029776,
                                                    -1.6613800909405605);
    cv::solvePnP(armor.type == SMALL ? SMALL_ARMOR_POINTS : 
                LARGE_ARMOR_POINTS, four_points,cameraMatrix,distCoeffs,
                rvec,tvec);
    std::cout << rvec << std::endl;
    std::cout << tvec << std::endl;
}