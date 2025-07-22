#ifndef _PNP_SOLVER_
#define _PNP_SOLVER_

#include <vector>

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "armor.hpp"

/**
 * @brief 解算PnP逻辑
 */
class PnPSolver
{
public:
    PnPSolver(sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo);
    bool solve(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec);

private:
    std::vector<cv::Point3f> m_armorPoints;
    cv::Mat m_cameraMatrix;
    cv::Mat m_distCoeffs;
};

#endif