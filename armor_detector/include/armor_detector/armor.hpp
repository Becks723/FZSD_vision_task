#ifndef _ARMOR_
#define _ARMOR_

#include <opencv2/opencv.hpp>

enum class ArmorColor
{
    Red,
    Blue,
};

enum class ArmorType
{
    Big,
    Small
};

struct Lightbar
{
    ArmorColor color;
    cv::Point2f center, top, bottom, top2bottom;
    double angle, angleError, length, ratio;

    Lightbar(const cv::RotatedRect& rect)
    {
        std::vector<cv::Point2f> corners(4);
        rect.points(&corners[0]);
        std::sort(corners.begin(), corners.end(), [](const cv::Point2f& a, const cv::Point2f& b)
        {
            return a.y < b.y;
        });

        center = rect.center;
        top = (corners[0] + corners[1]) / 2;
        bottom = (corners[1] + corners[3]) / 2;
        top2bottom = bottom - top;

        auto width = cv::norm(corners[0] - corners[1]);
        angle = std::atan2(top2bottom.y, top2bottom.x);
        angleError = std::abs(angle - CV_PI / 2);
        length = cv::norm(top2bottom);
        ratio = length / width;
    }
};  

struct Armor
{
    ArmorColor color;
    const Lightbar left, right;
    std::vector<cv::Point2f> points;

    cv::Point2f center;

    Armor(const Lightbar& left, const Lightbar& right)
        : left(left), right(right)
    {
        color = left.color;

        points.emplace_back(left.top);
        points.emplace_back(right.top);
        points.emplace_back(left.bottom);
        points.emplace_back(right.bottom);
    }
};

#endif