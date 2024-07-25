#pragma once
#include <deque>
#include <opencv4/opencv2/opencv.hpp>
#include "kalman.hpp"
#include <map>
#include <string>
#include "data.hpp"
#include <Eigen/Dense>
#include "pose.hpp"
#define INIT_RADIU 0.4

struct last_info
{
    double x = 0,y = 0,z = 0,yaw = 0;
    int last_frame = 0;
};

void fitCircle3D(std::deque<cv::Point3f>& points, 
    cv::Point3f& centerPoint, double& radius);

void init_kalman_filters(std::map<std::string,kalman>& kalman_filters,double delta);


void from_matrix_get_info(const Eigen::MatrixXd& X,TransformData& data);

void init_last_frame(std::map<std::string,int>& last_frame);

double get_distance(const Eigen::Vector3d& tvec,const last_info& last);
