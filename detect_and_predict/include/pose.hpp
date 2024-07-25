#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

class pose
{
public:
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
    pose() = default;
    pose(double x_,double y_,double z_,
        double yaw,double pitch,double roll);
    pose(const cv::Mat& tvec,const cv::Mat& rvec);
    pose(double x_,double y_,double z_,
        Eigen::Quaterniond quat_);
};


pose convert_from_a_to_b(const pose& from_a_to_object,
                        const pose& from_b_to_a);


