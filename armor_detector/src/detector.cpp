#include "armor_detector/detector.hpp"
#include "armor_detector/helpers.hpp"

using namespace cv;
using namespace std;

vector<Armor> Detector::detect(const Mat& bgr)
{
    // 转灰度图
    Mat gray;
    cvtColor(bgr, gray, COLOR_BGR2GRAY);

    // 二值化
    Mat binary;
    threshold(gray, binary, 100, 255, THRESH_BINARY);

    imshow("binary", binary);
    /*int key = waitKey(0);
    while (key != 'f')
        key = waitKey(0);*/

    // 获取轮廓点
    vector<vector<Point>> contours;
    findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    /*Mat drawContours = bgr.clone();
    for (const auto& contour : contours)
        helpers::drawPoints(drawContours, contour);
    imshow("drawContours", drawContours);*/

    // 获取灯条
    vector<Lightbar> lightbars;
    for (const auto& contour : contours)
    {
        auto rect = minAreaRect(contour);
        auto lightbar = Lightbar(rect);

        if (checkGeometry(lightbar))
            continue;
        lightbar.color = getArmorColor(bgr, contour);
        lightbars.emplace_back(lightbar);
    }

    // 将灯条从左到右排序
    sort(lightbars.begin(), lightbars.end(), [](const Lightbar& a, const Lightbar& b)
    { 
        return a.center.x < b.center.y; 
    });

    // 获取装甲板
    vector<Armor> armors;
    for (int i = 0; i < lightbars.size(); i++)
    {
        for (int j = i + 1; j < lightbars.size(); j++)
        {
            auto left = lightbars[i],
                right = lightbars[j];
            if (left.color != right.color) continue;

            Armor armor = Armor(left, right);

            armors.emplace_back(armor);
        }
    }

    // 显示装甲板灯条（Debug用）
    Mat drawArmors = bgr.clone();
    for (const auto& armor : armors)
    {
        helpers::drawPoints(drawArmors, armor.points);
    }
    imshow("draw_armors", drawArmors);
    waitKey(1);
    return armors;
}

ArmorColor Detector::getArmorColor(const cv::Mat& bgr, const std::vector<cv::Point>& contour)
{
    int redSum = 0,
        blueSum = 0;
    for (const auto& point : contour)
    {
        redSum += bgr.at<cv::Vec3b>(point)[2];
        blueSum += bgr.at<cv::Vec3b>(point)[0];
    }
    return blueSum > redSum 
        ? ArmorColor::Blue 
        : ArmorColor::Red;
}

bool Detector::checkGeometry(const Lightbar& lightbar)
{
    bool ratioOk = 2.0 <= lightbar.ratio && lightbar.ratio <= 20;
    bool angleOk = (lightbar.angleError * 57.3) < 45;
    return ratioOk && angleOk;
}