a#include <opencv4/opencv2/opencv.hpp>
#include <vector>
#include <math.h>
#include <armor.hpp>
#include <number_classifier.hpp>

int main(int argc,char** argv)
{
    std::string path;
    path += argv[1];
    path += ".jpg";
    cv::Mat image = cv::imread(path);
    Armor armor;
    armor.set_image(image);
    //number detect
    armor::NumberClassifier number_classifier(
        "model/mlp.onnx","model/label.txt",0.9);
    std::vector<Armor> vec_armor;
    vec_armor.push_back(armor);
    number_classifier.ExtractNumbers(image,vec_armor);
    number_classifier.Classify(vec_armor);
    std::cout << vec_armor[0].classification_result << std::endl;

    //pnp
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
    const std::vector<cv::Point3f> SMALL_ARMOR_POINTS = {
        { 0, +SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2},
        { 0, +SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
        { 0, -SMALL_ARMOR_WIDTH / 2, +SMALL_ARMOR_HEIGHT / 2},
        { 0, -SMALL_ARMOR_WIDTH / 2, -SMALL_ARMOR_HEIGHT / 2}
        };
    
    cv::Mat rvec, tvec;
    std::vector<cv::Point2f> points_2d;
    points_2d.push_back(armor.left_light.bottom);
    points_2d.push_back(armor.left_light.top);
    points_2d.push_back(armor.right_light.top);
    points_2d.push_back(armor.right_light.bottom);

    cv::solvePnP(SMALL_ARMOR_POINTS,points_2d,cameraMatrix,distCoeffs,rvec,tvec);
    std::cout << "rvec: " << rvec << std::endl << std::endl;
    std::cout << "tvec: " << tvec << std::endl;
    cv::Scalar color(0,255,0);
    cv::line(image,armor.left_light.bottom,armor.left_light.top,color);
    cv::line(image,armor.right_light.bottom,armor.right_light.top,color);
    cv::line(image,armor.left_light.bottom,armor.right_light.bottom,color);
    cv::line(image,armor.left_light.top,armor.right_light.top,color);
    //draw pic
    cv::imshow("image",image);
    cv::waitKey();
    return 0;
}