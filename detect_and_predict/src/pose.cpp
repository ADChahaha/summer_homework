#include "pose.hpp"

pose::pose(double x_,double y_,double z_,
            double yaw,double pitch,double roll):pos(x_,y_,z_)
{
    quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

pose::pose(double x_,double y_,double z_,
        Eigen::Quaterniond quat_):pos(x_,y_,z_)
{
    quat = quat_;
}

pose::pose(const cv::Mat& tvec,const cv::Mat& rvec) : pos(tvec.at<double>(0,0),
                                    tvec.at<double>(1,0),tvec.at<double>(2,0))
{
    quat = Eigen::AngleAxisd(rvec.at<double>(0,0), Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(rvec.at<double>(1,0), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(rvec.at<double>(2,0), Eigen::Vector3d::UnitX());
}

pose convert_from_a_to_b(const pose& from_a_to_object,
                        const pose& from_b_to_a)
{
    pose ret;
    ret.quat = from_b_to_a.quat * from_a_to_object.quat;
    ret.pos = from_b_to_a.quat * from_a_to_object.pos + from_b_to_a.pos;
    return ret;
}