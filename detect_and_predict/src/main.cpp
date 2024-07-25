#include "client.hpp"

int main(int argc,char** argv)
{
    Client client_camera_info(argv[1],5140);
    Client transform_request(argv[1],4399);
    double delta = 1 / 1.0 / std::stod(argv[3]);
    Client client_image(argv[1],std::stoi(argv[2]));
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
    pnp_solver solver(cameraMatrix,distCoeffs);
    //位姿变换
    int frame = 0;
    get_armor detecter;
    double radiu = INIT_RADIU;
    std::map<std::string,kalman> kalman_filters;
    init_kalman_filters(kalman_filters,delta);
    double yaw = 0;
    cv::Mat image;
    pose from_shooter_to_camera(pose(0.0,0.0,0.0,-M_PI / 2,0.0,-M_PI / 2));
    
    int cnt = 0;
    while(true)
    {
        
        auto transform_message = transform_request.receive_message();
        //std::cout << cnt++ << " haven't receive transform data!" << std::endl;
        sleep(1);
        if(transform_message.MessageType == TRANSFORM)
        {
            std::cout << "received transform data" << std::endl;
            TransformData data;
            memcpy(&data,transform_message.Data,sizeof(data));
            from_shooter_to_camera = pose(data.Translation[0],data.Translation[1],data.Translation[2],
            Eigen::Quaterniond(data.Rotation[3],data.Rotation[0],data.Rotation[1],data.Rotation[2]));   
            std::cout << from_shooter_to_camera.pos << std::endl << from_shooter_to_camera.quat << std::endl;
            break;
        }
        else if(transform_message.MessageType == STRING_MSG)
        {
            std::string str_msg((const char*)transform_message.Data,transform_message.DataLength);
                std::cout << "this is a message from server:\n" << 
                            str_msg << std::endl;
        }
        transform_request.send_message("Shooter","Camera");
    }

    std::map<int,std::vector<uchar>> images;
    std::map<std::string,last_info> lasts;
    lasts["1"];
    lasts["2"];
    lasts["3"];
    lasts["4"];
    lasts["5"];
    auto camera_message = client_camera_info.receive_message();
    if(camera_message.MessageType == CAMERA_INFO)
    {
        std::cout << "received camera info" << std::endl;
        CameraInfoData camera_info;
        memcpy(&camera_info,camera_message.Data, sizeof(CameraInfoData));
        solver.updateParam(cv::Mat(3,3,CV_64F,camera_info.CameraMatrix),
        cv::Mat(1,5,CV_64F,camera_info.DistortionCoefficients));
    }
    while(true)
    {
        auto message = client_image.receive_message();
        switch(message.MessageType)
        {      
            case IMAGE_MSG:
            {
                //todo封装为一个函数
                //如果是一张完整的图像,开始处理图像
                //如果不是一张完整的图像,继续读取
                std::vector<unsigned char>& image_vec = images[message.DataID];
                image_vec.insert(image_vec.end(),reinterpret_cast<const unsigned char*>(message.Data),
                reinterpret_cast<const unsigned char*>(message.Data) + message.DataLength);
                if((message.Offset + message.DataLength) == message.DataTotalLength)
                {
                    client_image.next_frame();
                    //完整地得到了这一帧图片
                    //提取出Mat
                    cv::Mat image = cv::imdecode(image_vec,cv::IMREAD_COLOR);
                    frame++;
                    //寻找装甲板  
                    auto armors = detecter.get_form_image(image,0.7);
                    std::set<std::string> se;
                    for(auto it = armors.begin();it != armors.end();)
                    {
                        if(se.find(it->number) == se.end())
                        {
                            se.insert(it->number);
                            it++;
                        }
                        else
                        {
                            it = armors.erase(it);
                        }
                    }
                    for(const auto& armor : armors)
                    {
                        cv::circle(image,armor.left_light.bottom,3,cv::Scalar(0,0,255),-1);
                        cv::circle(image,armor.left_light.top,3,cv::Scalar(0,0,255),-1);
                        cv::circle(image,armor.right_light.bottom,3,cv::Scalar(0,0,255),-1);
                        cv::circle(image,armor.right_light.top,3,cv::Scalar(0,0,255),-1);
                        std::cout << armor.left_light.bottom << std::endl;
                        std::cout << armor.left_light.top << std::endl;
                        std::cout << armor.right_light.bottom << std::endl;
                        std::cout << armor.right_light.top << std::endl;
                        std::string number = armor.number;
                        //pnp解算
                        cv::Mat rvec,tvec;
                        solver.solve(armor,rvec,tvec);
                        //位姿变换得到中心点xyz坐标
                        std::cout << "radiu: " << radiu << std::endl;
                        std::cout << "tvec:\n" << tvec << std::endl;
                        std::cout << "rvec:\n" << rvec << std::endl;
                        pose from_camera_to_center(convert_from_a_to_b(
                            pose(0,0,radiu,0,0,0),pose(tvec,rvec)));
                        //这个xyz是中心点的xyz坐标
                        pose from_shooter_to_center(convert_from_a_to_b(
                            from_camera_to_center,from_shooter_to_camera));
                        std::cout << "from camera to center:\n" << from_camera_to_center.pos << std::endl;
                        std::cout << "from shooter to camera:\n" << from_shooter_to_camera.quat << std::endl;
                        pose from_shooter_to_armor(convert_from_a_to_b(pose(tvec,rvec),from_shooter_to_camera));
                        //通过上一次的位置计算角速度和线速度和半径
                        std::cout << "from shooter to center:\n" << from_shooter_to_center.pos << std::endl; 
                        yaw = from_shooter_to_center.quat.
                        toRotationMatrix().eulerAngles(2, 1, 0)[0];
                        std::cout << "last yaw: " << lasts[number].yaw << std::endl;
                        std::cout << "yaw: " << yaw << std::endl;
                        double angle_v = (yaw - lasts[number].yaw) / 
                        (delta * (frame - lasts[number].last_frame));
                        double v = get_distance(from_shooter_to_armor.pos,lasts[number]) / 
                        (delta * (frame - lasts[number].last_frame));
                        std::cout << "frame: " << frame << std::endl;
                        std::cout << "last frame: " << lasts[number].last_frame << std::endl;
                        std::cout << "v: " << v << std::endl;
                        std::cout << "angle_v: " << angle_v << std::endl;
                        radiu = fabs(v / angle_v);
                        std::cout << "radiu: " << radiu << std::endl;
                        if((fabs(angle_v) < 5) && (fabs(angle_v) > 3) && ((fabs(v) < 2) && (fabs(v) > 1)))
                        {
                            Eigen::MatrixXd Z(4,1);
                            Z(0,0) = from_shooter_to_center.pos.x();
                            Z(1,0) = from_shooter_to_center.pos.y();
                            Z(2,0) = from_shooter_to_center.pos.z();
                            Z(3,0) = radiu;
                            //可靠的r   
                            kalman_filters[number].update(Eigen::MatrixXd::Zero(7,1),Z);
                        }
                        else
                        {
                            //不可靠的r
                            Eigen::MatrixXd Z(4,1);
                            auto X = kalman_filters[number].status();
                            Z(0,0) = from_shooter_to_center.pos.x();
                            Z(1,0) = from_shooter_to_center.pos.y();
                            Z(2,0) = from_shooter_to_center.pos.z();
                            Z(3,0) = X(6,0);
                            //可靠的r
                            kalman_filters[number].update(Eigen::MatrixXd::Zero(7,1),Z);
                        }
                        //更新卡尔曼
                        //如果线速度过大（突然的移动），直接xyz，yaw，r赋值，其余置0
                        //更新radiu(已更新)和last_info
                        lasts[number].x = from_shooter_to_armor.pos.x();
                        lasts[number].y = from_shooter_to_armor.pos.y();
                        lasts[number].z = from_shooter_to_armor.pos.z();
                        lasts[number].yaw = yaw;
                        lasts[number].last_frame = frame;
                        radiu = kalman_filters[number].status()(6,0);
                        std::cout << "my kalman:\n" << kalman_filters[number].status() << std::endl;
                        std::string str = "frame" + std::to_string(frame) + ":   " + 
                        "car" + number + " cur: " + "x: " +
                        std::to_string(from_shooter_to_center.pos.x()) + " y: " + 
                        std::to_string(from_shooter_to_center.pos.y()) + " z: " + 
                        std::to_string(from_shooter_to_center.pos.z()) + " r: " + 
                        std::to_string(radiu);
                        client_image.send_message(str);
                        auto X = kalman_filters[number].predict(Eigen::MatrixXd::Zero(7,1));
                        str = "frame" + std::to_string(frame) + ":   " + 
                        "car" + number + " predict: " + "x: " +
                        std::to_string(X(0,0)) + " y: " + 
                        std::to_string(X(1,0)) + " z: " + 
                        std::to_string(X(2,0)) + " r: " + 
                        std::to_string(X(6,0));
                    }
                    cv::namedWindow("image",cv::WINDOW_NORMAL);
                    cv::imshow("image",image);  
                }
                break;
            }
            case TRANSFORM:
            {
                TransformData data;
                memcpy(message.Data, &data, sizeof(TransformData));
                from_shooter_to_camera.pos.x() = data.Translation[0];
                from_shooter_to_camera.pos.y() = data.Translation[1];
                from_shooter_to_camera.pos.z() = data.Translation[2];
                from_shooter_to_camera.quat.coeffs() << data.Rotation[0],
                data.Rotation[1], data.Rotation[2], data.Rotation[3]; 
                std::cout << "got transform data!" << std::endl;
                break;
            }         
            case CAMERA_INFO:
            {
                CameraInfoData camera_info;
                memcpy(&camera_info,message.Data, sizeof(CameraInfoData));
                solver.updateParam(cv::Mat(3,3,CV_64F,camera_info.CameraMatrix),
                cv::Mat(1,5,CV_64F,camera_info.DistortionCoefficients));
                break;
            }   
            case STRING_MSG:
            {
                std::string str_msg((const char*)message.Data,message.DataLength);
                std::cout << "this is a message from server:\n" << 
                            str_msg << std::endl;
                break;
            }    
            default:
            {
                break;
            }
        }   
    }
    

    return 0;
}


























