#include <iostream>
#include "pose.hpp"

int main()
{
    while(1)
    {
        Eigen::Quaterniond q1(0.5,-0.5,0.5,-0.5);
        Eigen::Quaterniond q2(0,0.7,-0.13,1.9);
        std::cout << (q1 * q2) * q1.conjugate();
        double x,y,z,yaw,pitch,roll,x_gimbal_to_camera,x_gimbal_to_shooter,z_gimbal_to_shooter;
        std::cout << "请输入装甲板想对于相机的x y z yaw pitch roll" << std::endl;
        std::cin >> x >> y >> z >> yaw >> pitch >> roll;
        std::cout << "请输入想对于云台坐标系,相机的偏移量x" << std::endl;
        std::cin >> x_gimbal_to_camera;
        std::cout << "请输入想对于云台坐标系,枪管的偏移量x z" << std::endl;
        std::cin >> x_gimbal_to_shooter >> z_gimbal_to_shooter;
        pose camera(x,y,z,yaw,pitch,roll);
        std::cout << "my camera is:\n" << camera.pos << "\n" << camera.quat << std::endl;                    
        pose gimbal(convert_from_a_to_b(camera,pose(x_gimbal_to_camera,0,0,-1.57,0,-1.57)));
        std::cout << "my gimbal is:\n" << gimbal.pos << "\n" << gimbal.quat << std::endl;   
    }
    
    return 0;
}


//傅立叶变换
//yaw 90 pitch 0 roll 90