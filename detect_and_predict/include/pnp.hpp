#pragma once
#include <armor.hpp>

class pnp_solver
{
private:
    //相机内参
    //畸变矩阵
    //大小装甲板的大小
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
        { 0, +SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2},
        { 0, +SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
        { 0, -SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
        { 0, -SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2}
        };
    const std::vector<cv::Point3f> LARGE_ARMOR_POINTS = {
        { 0, +LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2},
        { 0, +LARGE_ARMOR_WIDTH / 2, +LARGE_ARMOR_HEIGHT / 2},
        { 0, -LARGE_ARMOR_WIDTH / 2, +LARGE_ARMOR_HEIGHT / 2},
        { 0, -LARGE_ARMOR_WIDTH / 2, -LARGE_ARMOR_HEIGHT / 2}
        };
public:
    pnp_solver() = default;
    pnp_solver(const cv::Mat& camera,const cv::Mat& dist);
    void updateParam(const cv::Mat& camera,const cv::Mat& dist);
    void solve(const Armor& armor,cv::Mat& rvec,cv::Mat& tvec);
};