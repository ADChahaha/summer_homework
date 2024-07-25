#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

int main(int argc, char** argv) {
    // 读取彩色图像并转换为灰度图像
    cv::Mat image = cv::imread(argv[1]);
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    cv::imshow("image", image);

    // 缩放到最适合计算的大小
    cv::Mat padded;
    int m = cv::getOptimalDFTSize(image.rows);
    int n = cv::getOptimalDFTSize(image.cols);
    cv::copyMakeBorder(image, padded, 0, m - image.rows, 0, n - image.cols, 
                       cv::BORDER_CONSTANT, cv::Scalar::all(0));

    // 转换为 float32 的图像并创建复数图像
    cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
    cv::Mat complexI;
    cv::merge(planes, 2, complexI);

    // 离散傅立叶变换
    cv::dft(complexI, complexI);

    // 计算模
    cv::split(complexI, planes);
    cv::magnitude(planes[0], planes[1], planes[0]);
    cv::Mat mag = planes[0];

    // 取对数适合显示
    mag += cv::Scalar::all(1);
    cv::log(mag, mag);
    // 裁剪频谱图像
    mag = mag(cv::Rect(0, 0, mag.cols & -2, mag.rows & -2));

    // 重新排列傅立叶图像中的象限，使得原点位于图像中心
    int cx = mag.cols / 2;
    int cy = mag.rows / 2;

    cv::Mat q0(mag, cv::Rect(0, 0, cx, cy));
    cv::Mat q1(mag, cv::Rect(cx, 0, cx, cy));
    cv::Mat q2(mag, cv::Rect(0, cy, cx, cy));
    cv::Mat q3(mag, cv::Rect(cx, cy, cx, cy));


    cv::normalize(mag, mag, 0, 1, cv::NORM_MINMAX);

    cv::imshow("image", image);
    cv::imshow("fourier", mag);

    int cutoff = std::stoi(argv[2]); // 中心圆半径（像素）//默认中间为高频
    cv::Mat mask = cv::Mat::zeros(complexI.size(), CV_8U);
    cv::circle(mask, cv::Point(cx, cy), cutoff, cv::Scalar(255), -1);
    cv::circle(mask, cv::Point(cx, cy), cutoff - 1, cv::Scalar(0), -1);
    cv::Mat_<float> realI, imagI;
    cv::split(complexI, planes);
    planes[0].copyTo(realI, mask);
    planes[1].copyTo(imagI, mask);
    cv::merge(std::vector<cv::Mat>{realI, imagI}, complexI);

    cv::idft(complexI, complexI);

    // 获取逆变换的实部
    cv::split(complexI, planes);
    cv::Mat restoredImage;
    cv::normalize(planes[0], restoredImage, 0, 255, cv::NORM_MINMAX);
    restoredImage.convertTo(restoredImage, CV_8U);
    //cv::threshold(restoredImage,restoredImage,50,255,cv::THRESH_BINARY_INV);
    //restoredImage = 255 - restoredImage;
    // 显示原始图像和处理后的图像
    cv::imshow("Restored Image", restoredImage);
    cv::Mat res = image - restoredImage;
    cv::threshold(res,res,1,255,cv::THRESH_BINARY);
    cv::imshow("res",res);
    // 显示原图像和频谱图像
    

    cv::waitKey();
    return 0;
}
