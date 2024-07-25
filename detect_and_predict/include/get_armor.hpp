#pragma once
#include "armor.hpp"
#include "number_classifier.hpp"

#define HEIGHT 0.055
#define WIDTH 0.008
#define RATIO (HEIGHT / WIDTH)
#define NOISE_POINTS 20
#define LIGHT_ERROR_BOTTOM_RANGE 0.7
#define SMALL_ARMOR_BOTTOM_ERROR_RANGE 0.25
#define LARGE_ARMOR_TOP_ERROR_RANGE 0.175
#define LARGE_ARMOR_BOTTOM_ERROR_RANGE 0.4
#define SMALL_ARMOR_TOP_ERROR_RANGE 0.4
#define THRESHOLD_VALUE 30
#define PARALLEL_ERROR_RANGE 8
#define MODEL_PATH "asset/model/mlp.onnx"
#define LABEL_PATH "asset/model/label.txt"
#include <memory>

class get_armor
{
public:
    get_armor() = default;
    std::vector<Armor> get_form_image(cv::Mat& image,double net_threshold_value);
private:
    void getClockwiseOrderedPoints(const cv::RotatedRect& minRect, cv::Point2f orderedPoints[4]);
};