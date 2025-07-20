#include <armor_detector/pnpsolver.hpp>

PnPSolver::PnPSolver(sensor_msgs::msg::CameraInfo::ConstSharedPtr cameraInfo)
{
    const double ARMOR_WIDTH = 140 / 1000.0;  // 单位: m
    const double ARMOR_HEIGHT = 125 / 1000.0;

    // 相对于电脑显示屏：z向前 x向右 y向下
    // 从左上开始 顺时针
    m_armorPoints.emplace_back(cv::Point3f(-ARMOR_WIDTH / 2, -ARMOR_HEIGHT / 2, 0));
    m_armorPoints.emplace_back(cv::Point3f(ARMOR_WIDTH / 2, -ARMOR_HEIGHT / 2, 0));
    m_armorPoints.emplace_back(cv::Point3f(ARMOR_WIDTH / 2, ARMOR_HEIGHT / 2, 0));
    m_armorPoints.emplace_back(cv::Point3f(-ARMOR_WIDTH / 2, ARMOR_HEIGHT / 2, 0));

    // 初始化相机内参
    m_cameraMatrix = cv::Mat(3, 3, CV_64F, const_cast<double*>(cameraInfo->k.data())).clone();
    m_distCoeffs = cv::Mat(1, 5, CV_64F, const_cast<double*>(cameraInfo->d.data())).clone();
}

bool PnPSolver::solve(const Armor& armor, cv::Mat& rvec, cv::Mat& tvec)
{
    std::vector<cv::Point2f> imgPoints;
    imgPoints.emplace_back(armor.left.top);
    imgPoints.emplace_back(armor.right.top);
    imgPoints.emplace_back(armor.right.bottom);
    imgPoints.emplace_back(armor.left.bottom);

    return cv::solvePnP(m_armorPoints, imgPoints, m_cameraMatrix, m_distCoeffs, rvec, tvec);
}