#ifndef _HELPERS_
#define _HELPERS_

#include <vector>
#include <opencv2/opencv.hpp>

namespace helpers
{
    /**
     * @brief 工具函数。cv图像上画点
     * 
     * @param img 
     * @param points 
     * @param color 
     * @param thickness 
     */
    inline void drawPoints(const cv::Mat& img, 
        const std::vector<cv::Point>& points,
        const cv::Scalar& color = cv::Scalar(0, 0, 255),
        int thickness = 2)
    {
        using std::vector;

        vector<vector<cv::Point>> contours;
        contours.emplace_back(points);
        cv::drawContours(img, contours, -1, color, thickness);
    }

    /**
     * @brief 工具函数。cv图像上画点
     * 
     * @param img 
     * @param points 
     * @param color 
     * @param thickness 
     */
    inline void drawPoints(const cv::Mat& img, 
        const std::vector<cv::Point2f>& points,
        const cv::Scalar& color = cv::Scalar(0, 0, 255),
        int thickness = 2)
    {
        using std::vector;

        vector<cv::Point> intPoints(points.begin(), points.end());
        drawPoints(img, intPoints, color, thickness);
    }

    /**
     * @brief 工具函数。cv图像上加文字
     * 
     * @param img 
     * @param points 
     * @param color 
     * @param thickness 
     */
    inline void drawText(const cv::Mat& img,
        const std::string& text,
        const cv::Point& position,
        double scale = 1.0,
        const cv::Scalar& color = cv::Scalar(0, 255, 255),
        int thickness = 2)
    {
        cv::putText(img, text, position, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness);
    }
} // namespace helpers

#endif