#pragma once
#include <opencv4/opencv2/opencv.hpp>
#include <string>
#include <algorithm>
#include "handle_algorithm.hpp"
#include "armor.hpp"

#define SMALL_ARMOR_WIDTH 0.135
#define SMALL_ARMOR_HEIGHT 0.055
#define LARGE_ARMOR_WIDTH 0.225
#define LARGE_ARMOR_HEIGHT 0.055

class light
{
public:
    cv::Point2f bottom;
    cv::Point2f top;
};

enum ArmorType 
{
    SMALL,    
    LARGE
};

class Armor
{
public:
    light left_light,right_light;  //灯条
    cv::Mat number_image;     //数字ROI
    std::string number;    //数字
    ArmorType type;      //装甲板大小
    double classification_confidence; //可信任度
    std::string classification_result;  //打印的结果
public:
    Armor() = default;
    void getclockwisepoints(cv::Point2f four_points[4]);
};