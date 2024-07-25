#include "handle_algorithm.hpp"

void fitCircle3D(std::deque<cv::Point3f>& points, cv::Point3f& centerPoint, double& radius) {
    if (points.size() != 3) {
        throw std::invalid_argument("Exactly three points are required to fit a circle in 3D.");
    }

    // Extract the three points
    cv::Point3f p1 = points[0];
    cv::Point3f p2 = points[1];
    cv::Point3f p3 = points[2];

    // 检查三个点是否重合或共线
    if ((p1 == p2 && p2 == p3) || 
        (cv::norm(p2 - p1) == 0) ||
        (cv::norm(p3 - p1) == 0) ||
        (cv::norm(p3 - p2) == 0)) {
        centerPoint = cv::Point3f(0, 0, 0);
        radius = 0.0;
        return;
    }

    // Calculate the midpoints of the segments p1p2 and p2p3
    cv::Point3f mid1 = (p1 + p2) * 0.5;
    cv::Point3f mid2 = (p2 + p3) * 0.5;

    // Calculate the normal vectors of the planes formed by p1, p2 and p2, p3
    cv::Point3f normal1 = (p2 - p1).cross(p3 - p1);
    cv::Point3f normal2 = (p3 - p2).cross(p1 - p2);

    // 如果法向量为零，表示点共线
    if (cv::norm(normal1) == 0 || cv::norm(normal2) == 0) {
        centerPoint = cv::Point3f(0, 0, 0);
        radius = 0.0;
        return;
    }

    // Define the direction vectors of the perpendicular bisectors
    cv::Point3f dir1 = normal1.cross(p2 - p1);
    cv::Point3f dir2 = normal2.cross(p3 - p2);

    // Create matrices for solving the linear system
    cv::Mat A(2, 3, CV_64F);
    cv::Mat B(2, 1, CV_64F);

    // Fill the matrices
    A.at<double>(0, 0) = dir1.x; A.at<double>(0, 1) = dir1.y; A.at<double>(0, 2) = dir1.z;
    A.at<double>(1, 0) = dir2.x; A.at<double>(1, 1) = dir2.y; A.at<double>(1, 2) = dir2.z;

    B.at<double>(0, 0) = dir1.dot(mid1);
    B.at<double>(1, 0) = dir2.dot(mid2);

    // Solve for the center
    cv::Mat center;
    if (!cv::solve(A, B, center, cv::DECOMP_SVD)) {
        centerPoint = cv::Point3f(0, 0, 0);
        radius = 0.0;
        return;
    }

    centerPoint.x = center.at<double>(0, 0);
    centerPoint.y = center.at<double>(1, 0);
    centerPoint.z = center.at<double>(2, 0);

    // Calculate the radius
    radius = cv::norm(centerPoint - p1);
}


void init_kalman_filters(std::map<std::string,kalman>& kalman_filters,double delta)
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(7,7);
    A(0,3) = delta;
    A(1,4) = delta;
    A(2,5) = delta;
    Eigen::MatrixXd B = Eigen::MatrixXd::Identity(7,7);
    Eigen::MatrixXd H(4,7);
    H(0,0) = 1.0;
    H(1,1) = 1.0;
    H(2,2) = 1.0;
    H(3,6) = 1.0;
    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(7,7);
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(4,4) * 10;
    Eigen::MatrixXd P_K0 = Eigen::MatrixXd::Identity(7,7) * 10;
    Eigen::MatrixXd X_K0 = Eigen::MatrixXd::Zero(7,1);
    X_K0(6,0) = INIT_RADIU;
    kalman kalman_filter(A,B,R,Q,P_K0,X_K0,H);
    kalman_filters["1"] = kalman_filter;
    kalman_filters["2"] = kalman_filter;
    kalman_filters["3"] = kalman_filter;
    kalman_filters["4"] = kalman_filter;
    kalman_filters["5"] = kalman_filter;
}



void from_matrix_get_info(const Eigen::MatrixXd& X,TransformData& data)
{
    //假设X是一个9行1列的向量
    //位置
    data.Translation[0] = X(0,0);
    data.Translation[1] = X(1,0);
    data.Translation[2] = X(2,0);
    //角度
    double yaw = X(6,0);
    double pitch = X(7,0);
    double roll = X(8,0);
    Eigen::Quaterniond quat;
    quat = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    data.Rotation[0] = quat.w();
    data.Rotation[1] = quat.x();
    data.Rotation[2] = quat.y();
    data.Rotation[3] = quat.z();
}

void init_last_frame(std::map<std::string,int>& last_frame)
{
    last_frame["1"] = 0;
    last_frame["2"] = 0;
    last_frame["3"] = 0;
    last_frame["4"] = 0;
    last_frame["5"] = 0;
}

double get_distance(const Eigen::Vector3d& tvec,const last_info& last)
{
    cv::Mat mat_tvec = (cv::Mat_<double>(1,3) << tvec[0], tvec[1], tvec[2]);
    cv::Mat last_tvec = (cv::Mat_<double>(1,3) << last.x, last.y, last.z);
    return cv::norm(mat_tvec,last_tvec);
}