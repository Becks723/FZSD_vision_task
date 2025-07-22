#ifndef _DETECTOR_
#define _DETECTOR_

#include <list>

#include "armor.hpp"

struct DetectorConfig
{
    int binary_thres;
    /* 灯条高宽比 */
    double lightbar_min_ratio;
    double lightbar_max_ratio;
    /* 灯条角度 */
    double lightbar_angle;
};

class Detector
{
public:
    Detector(const std::string& pkgShareDir, const DetectorConfig& config);
    std::list<Armor> detect(const cv::Mat& frame);

private:
    bool checkGeometry(const Lightbar& lightbar);
    bool checkGeometry(const Armor& armor);
    bool checkName(const Armor& armor);
    ArmorColor getArmorColor(const cv::Mat& bgr, const std::vector<cv::Point>& contour);
    cv::Mat getPattern(const cv::Mat& bgr_img, const Armor& armor);
    void classify(Armor& armor);

    const std::string m_pkgShareDir;
    const DetectorConfig m_config;
};

#endif