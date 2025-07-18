#ifndef _DETECTOR_
#define _DETECTOR_

#include <list>

#include "armor.hpp"

class Detector
{
public:
    Detector(const std::string& pkgShareDir);
    std::vector<Armor> detect(const cv::Mat& frame);

private:
    bool checkGeometry(const Lightbar& lightbar);
    bool checkGeometry(const Armor& armor);
    bool checkName(const Armor& armor);
    ArmorColor getArmorColor(const cv::Mat& bgr, const std::vector<cv::Point>& contour);
    cv::Mat getPattern(const cv::Mat& bgr_img, const Armor& armor);
    void classify(Armor& armor);

    const std::string m_pkgShareDir;
};

#endif