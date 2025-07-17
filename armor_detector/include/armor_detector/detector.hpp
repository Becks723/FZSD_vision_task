#ifndef _DETECTOR_
#define _DETECTOR_

#include <list>

#include "armor.hpp"

class Detector
{
public:
    std::vector<Armor> detect(const cv::Mat& frame);

private:
    bool checkGeometry(const Lightbar& lightbar);
    ArmorColor getArmorColor(const cv::Mat& bgr, const std::vector<cv::Point>& contour);
};

#endif