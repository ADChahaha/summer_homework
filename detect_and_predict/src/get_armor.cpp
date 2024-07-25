#include <get_armor.hpp>

void getclockwisepoints(cv::Point2f four_points[4])
{
    std::sort(four_points,four_points+4,[](const cv::Point2f& a,const cv::Point2f& b)
    {
        return a.y < b.y;
    });
    if(four_points[0].x > four_points[1].x)
        std::swap(four_points[0],four_points[1]);
    if(four_points[2].x > four_points[3].x)
        std::swap(four_points[2],four_points[3]);
    std::swap(four_points[0],four_points[2]);
    std::swap(four_points[2],four_points[1]);
}

std::vector<Armor> get_armor::get_form_image(cv::Mat& image,double net_threshold_value)
{
    cv::Mat green_channel(image.rows,image.cols,CV_8UC1);
    cv::Mat copy;
    static int from_to[] = {1, 0};  // 从绿色通道到单通道
    cv::mixChannels(&image, 1, &green_channel, 1, from_to, 1);
    // 定义结构元素（内核）
    int morph_size = 5; // 调整此值来改变结构元素的大小
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * morph_size + 1, 2 * morph_size + 1),
                                                cv::Point(morph_size, morph_size));
    cv::morphologyEx(green_channel, green_channel, cv::MORPH_CLOSE, element);
    //cv::extractChannel(image,green_channel,1);
    //cv::cvtColor(image,green_channel,cv::COLOR_BGR2GRAY);
    cv::threshold(green_channel,green_channel,THRESHOLD_VALUE,255,cv::THRESH_BINARY);
    //cv::erode(green_channel,green_channel,cv::Mat(),cv::Point(-1,-1),1);
    //cv::imshow("gray",green_channel);
    //寻找轮廓
    std::vector<std::vector<cv::Point>> points;
    cv::findContours(green_channel,points,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE); 
    //确定里面是竖着的灯条
    //把不是灯条的东西全部杀掉
    std::vector<light> lights;
    //获得灯条
    int cnt = 0;
    for (auto it = points.begin(); it != points.end();) 
    {
        //删去点少的噪声
        if(it->size() < NOISE_POINTS)
        {
            it = points.erase(it);
            continue;
        }
        // 计算当前轮廓的凸包
        std::vector<cv::Point> convexHullPoints;
        cv::convexHull(*it, convexHullPoints);
        //获得最小矩形
        cv::RotatedRect minRect = cv::minAreaRect(convexHullPoints);
        //对于每个矩形,确定上下部分
        light one_light;
        cv::Point2f four_points[4];
        minRect.points(four_points);
        getclockwisepoints(four_points);
        one_light.top = (four_points[2] + four_points[1]) / 2;
        one_light.bottom = (four_points[0] + four_points[3]) / 2;
        //计算height和width
        //根据长宽比杀掉不是灯条的
        double height,width;
        
        height = sqrt((four_points[0].x - four_points[1].x) * (four_points[0].x - four_points[1].x) +
                     (four_points[0].y - four_points[1].y) * (four_points[0].y - four_points[1].y));
        width = sqrt((four_points[2].x - four_points[1].x) * (four_points[2].x - four_points[1].x) + 
                    (four_points[2].y - four_points[1].y) * (four_points[2].y - four_points[1].y));
        double ratio = height / width;
        
        //长宽比在可容许范围内,认为是灯条
        if(ratio > (1 - LIGHT_ERROR_BOTTOM_RANGE) * RATIO)
        {
            lights.push_back(one_light);
            cv::circle(image,one_light.bottom,1,cv::Scalar(0,255,0),-1);
            cv::circle(image,one_light.top,1,cv::Scalar(0,255,0),-1);
            cv::putText(image,std::to_string(ratio),cv::Point2f(one_light.bottom.x,one_light.bottom.y + 200),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0));
        }
        it++;
    }
    //通过灯条来计算装甲板
    //将灯条从左往右排序
    std::sort(lights.begin(),lights.end(),[](const light& a,const light& b)
    {
        return a.bottom.x < b.bottom.x;
    });
    //生成装甲板
    std::vector<Armor> armors;
    if(0 == lights.size())
        return armors;
    for(int i = 0,j;i < lights.size() - 1;++i)
    {
        //装甲板左右灯条
        Armor armor;
        armor.left_light = lights[i];
        //如果和下一条的y差距过大,换成再下一条
        for(j = i + 1;j < lights.size();++j)
        {
            armor.right_light = lights[j];
            if(fabs(armor.right_light.top.y - armor.left_light.top.y) < 
            fabs(armor.left_light.top.y - armor.left_light.bottom.y) / 3)
                break;
        }
        if(j == lights.size())
            continue;
        //判断灯条是否平行
        double left_angle = atan2(armor.left_light.top.y - armor.left_light.bottom.y,
                                armor.left_light.top.x - armor.left_light.bottom.x);
        double right_angle = atan2(armor.right_light.top.y - armor.right_light.bottom.y,
                                armor.right_light.top.x - armor.right_light.bottom.x);
        double angleDifference = fabs(left_angle - right_angle) * 180 / M_PI;
        //cv::putText(image,std::to_string(angleDifference),cv::Point2f(armor.left_light.bottom.x,armor.left_light.bottom.y + 200),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0));
        if(angleDifference > PARALLEL_ERROR_RANGE)
            continue;
        //根据长宽比确定大小装甲板
        double width,height;
        height = sqrt((armor.left_light.bottom.x - armor.left_light.top.x) * 
                    (armor.left_light.bottom.x - armor.left_light.top.x) +
                    (armor.left_light.bottom.y - armor.left_light.top.y) *
                    (armor.left_light.bottom.y - armor.left_light.top.y));
        width = sqrt((armor.left_light.top.x - armor.right_light.top.x) * 
                    (armor.left_light.top.x - armor.right_light.top.x) + 
                    (armor.left_light.top.y - armor.right_light.top.y) * 
                    (armor.left_light.top.y - armor.right_light.top.y));
        double ratio = height / width;
        double SMALL_ARMOR_RATIO = SMALL_ARMOR_HEIGHT / SMALL_ARMOR_WIDTH;
        double LARGE_ARMOR_RATIO = LARGE_ARMOR_HEIGHT / LARGE_ARMOR_WIDTH;
        cv::putText(image,std::to_string(ratio),cv::Point2f(armor.left_light.bottom.x,armor.left_light.bottom.y + 250),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0));
        //判断大小装甲板,如果完全不符合,直接下一个
        if(ratio / SMALL_ARMOR_RATIO < (1 + SMALL_ARMOR_TOP_ERROR_RANGE) && 
            ratio / SMALL_ARMOR_RATIO > (1 - SMALL_ARMOR_BOTTOM_ERROR_RANGE))
            {
                armor.type = SMALL;
                armors.push_back(armor);
            }
        else if(ratio / LARGE_ARMOR_RATIO < (1 + LARGE_ARMOR_TOP_ERROR_RANGE) && 
            ratio / LARGE_ARMOR_RATIO > (1 - LARGE_ARMOR_BOTTOM_ERROR_RANGE))
            {
                armor.type = LARGE;
                armors.push_back(armor);
            }
    }
    for(auto i :armors)
    {
        cv::putText(image,"using net",cv::Point2f(i.left_light.bottom.x,i.left_light.bottom.y + 300),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0));
    }
    std::vector<std::string> ignore_classes = {"negative"};
    //把可能是装甲板的东西扔到number_classify里面
    armor::NumberClassifier number_classifier(MODEL_PATH,LABEL_PATH,
                                            net_threshold_value,ignore_classes);
    number_classifier.ExtractNumbers(image,armors);
    number_classifier.Classify(armors);
    // for(auto i :armors)
    // {
    //     cv::putText(image,"good",cv::Point2f(i.left_light.bottom.x,i.left_light.bottom.y + 500),cv::FONT_HERSHEY_COMPLEX,1,cv::Scalar(0,255,0));
    // }
    return armors;
}