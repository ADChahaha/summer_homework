#include <Eigen/Dense>
#include <iostream>

class pose
{
public:
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
    pose() = default;
    pose(const Eigen::Vector3d& pos_,const Eigen::Quaterniond& quat_);
    pose(double x_,double y_,double z_,const Eigen::Quaterniond& quat_);
    pose(double x_,double y_,double z_,
        double quat_w,double quat_x,double quat_y,double quat_z);
    pose(const Eigen::Vector3d& pos_,
        double quat_w,double quat_x,double quat_y,double quat_z);
    pose(double x_,double y_,double z_,
        double yaw,double pitch,double roll);
    pose(const Eigen::Vector3d& pos_,
        double yaw,double pitch,double roll);
};


pose convert_from_a_to_b(const pose& from_a_to_object,
                        const pose& from_b_to_a);


