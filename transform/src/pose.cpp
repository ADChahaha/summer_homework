#include "pose.hpp"


pose::pose(const Eigen::Vector3d& pos_,const Eigen::Quaterniond& quat_) : 
                                        pos(pos_),quat(quat_){}

pose::pose(double x_,double y_,double z_,const Eigen::Quaterniond& quat_):
                                        pos(x_,y_,z_),quat(quat_){}                    
pose::pose(double x_,double y_,double z_,
    double quat_w,double quat_x,double quat_y,double quat_z):
    pos(x_,y_,z_),quat(quat_w,quat_x,quat_y,quat_z){}
pose::pose(const Eigen::Vector3d& pos_,
    double quat_w,double quat_x,double quat_y,double quat_z):
    pos(pos_),quat(quat_w,quat_x,quat_y,quat_z){}
pose::pose(double x_,double y_,double z_,
            double yaw,double pitch,double roll):pos(x_,y_,z_)
{
    quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}
pose::pose(const Eigen::Vector3d& pos_,
    double yaw,double pitch,double roll):pos(pos_)
{
    quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

pose convert_from_a_to_b(const pose& from_a_to_object,
                        const pose& from_b_to_a)
{
    pose ret;
    std::cout << from_b_to_a.pos << std::endl;
    ret.quat = from_b_to_a.quat * from_a_to_object.quat;
    ret.pos = from_b_to_a.quat * from_a_to_object.pos + from_b_to_a.pos;
    return ret;
}