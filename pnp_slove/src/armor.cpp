#include <armor.hpp>

int clockwiseComparator(const void *a, const void *b, void *center_ptr) {
    cv::Point2f center = *(cv::Point2f*)center_ptr;
    cv::Point2f pt1 = *(cv::Point2f*)a;
    cv::Point2f pt2 = *(cv::Point2f*)b;

    double angle1 = atan2(pt1.y - center.y, pt1.x - center.x);
    double angle2 = atan2(pt2.y - center.y, pt2.x - center.x);

    if (angle1 < angle2) return -1;
    if (angle1 > angle2) return 1;
    return 0;
}
void Armor::set_image(cv::Mat& image)
{
    std::vector<cv::Mat> channel;
    //blue channel
    cv::Mat gray;
    cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);
    cv::threshold(gray,gray,180,255,cv::THRESH_BINARY);
    cv::imshow("1",gray);

   // cv::Mat dst,corner;
    //cv::cornerHarris(gray,dst,2,3,0.04);
   //dst *= 300;
    //dst.convertTo(corner, CV_8U, 255.0, 0);
    //cv::threshold(corner,corner,100,255,cv::THRESH_BINARY);
    //cv::dilate(corner,corner,cv::Mat(),cv::Point(-1,-1),5);
    std::vector<std::vector<cv::Point>> points;
    cv::findContours(gray,points,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);
    //get two contours
    for(auto it = points.begin();it != points.end();)
    {
        if(it->size() < 20)
        {
            it = points.erase(it);
        }
        else
            it++;
    }

    //wait xiugai
    //make sure only two or four
    std::vector<cv::RotatedRect> rect;
    for (size_t i = 0; i < points.size(); ++i) 
    {
        // 计算当前轮廓的凸包
        std::vector<cv::Point> convexHullPoints;
        cv::convexHull(points[i], convexHullPoints);
        cv::RotatedRect minRect = cv::minAreaRect(convexHullPoints);
        rect.push_back(minRect);
    }
    cv::Point2f left[4],right[4];
    cv::Point2f center_left,center_right;
    if(rect[0].center.x < rect[1].center.x)
    {
        center_left = rect[0].center;
        center_right = rect[1].center;
        rect[0].points(left);
        rect[1].points(right);  
    }
    else
    {
        center_left = rect[1].center;
        center_right = rect[0].center;
        rect[0].points(right);
        rect[1].points(left);  
    }
    
    std::sort(left, left + 4, [&center_left](const cv::Point2f& a, const cv::Point2f& b) {
        return std::atan2(a.y - center_left.y, a.x - center_left.x) < std::atan2(b.y - center_left.y, b.x - center_left.x);
    });
    std::sort(right, right + 4, [&center_right](const cv::Point2f& a, const cv::Point2f& b) {
        return std::atan2(a.y - center_right.y, a.x - center_right.x) < std::atan2(b.y - center_right.y, b.x - center_right.x);
    });
    
    left_light.bottom = left[3];
    left_light.top = left[0];
    right_light.top = right[1];
    right_light.bottom = right[2];
    
    //左下角顺时针顺序
    
    
    //根据长宽比确定大小装甲板
    double width,height;
    height = sqrt((left_light.bottom.x - left_light.top.x) * 
                (left_light.bottom.x - left_light.top.x) +
                (left_light.bottom.y - left_light.top.y) *
                (left_light.bottom.y - left_light.top.y));
    width = sqrt((left_light.top.x - right_light.top.x) * 
                (left_light.top.x - right_light.top.x) + 
                (left_light.top.y - right_light.top.y) * 
                (left_light.top.y - right_light.top.y));
    double proportional = width / height;
    double small_armor_proportional = SMALL_ARMOR_WIDTH / SMALL_ARMOR_HEIGHT;
    if(proportional / small_armor_proportional < 1.1 && 
        proportional / small_armor_proportional > 0.9)
        type = SMALL;
    else
        type = LARGE;
}