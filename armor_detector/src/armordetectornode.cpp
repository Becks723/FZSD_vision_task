#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/format.h>

#include "armor_detector/detector.hpp"
#include "armor_detector/helpers.hpp"

#define DEBUG 0

class ArmorDetectorNode : public rclcpp::Node
{
public:
    ArmorDetectorNode()
        : Node("armor_detector"), 
        m_pkgShareDir(ament_index_cpp::get_package_share_directory("armor_detector")),
        m_detector(std::make_unique<Detector>(m_pkgShareDir))
    {
        RCLCPP_INFO(get_logger(), "Starting armor_detector node!");

        m_camSub = image_transport::create_camera_subscription(
            this,
            "image_raw",
            std::bind(&ArmorDetectorNode::subscriptionCallback, 
                this, std::placeholders::_1, std::placeholders::_2),
            "raw"
        );

#if DEBUG
        Detector detector;
        cv::VideoCapture video(
            shareDir + "/assets/8radps.avi");
        while (true)
        {
            cv::Mat frame;
            video >> frame;

            auto armors = detector.detect(frame);

            cv::Mat draw_armor = frame.clone();
            int index = 0;
            for (const Armor& armor : armors)
            {
                helpers::drawText(
                    draw_armor,
                    fmt::format("armor{}", index++),
                    armor.center
                );
            }
            cv::imshow("draw_armor", draw_armor);
            auto key = cv::waitKey(25);
            if (key == 'q')
                break;
        }
        return;
#endif
    }

private:
    void subscriptionCallback(const sensor_msgs::msg::Image::ConstSharedPtr& image,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info)
    {
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvShare(image, "bgr8")->image;
        }
        catch(const cv_bridge::Exception& ex)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", ex.what());
            return;
        }

        if (frame.empty())
        {
            RCLCPP_INFO(get_logger(), "Null frame!");
            return;
        }

        auto armors = m_detector->detect(frame);

        RCLCPP_INFO(get_logger(), "%d armors detected.", armors.size());

        cv::Mat draw_armor = frame.clone();
        for (const Armor& armor : armors)
        {
            helpers::drawPoints(draw_armor, armor.points);
            helpers::drawText(
                draw_armor,
                fmt::format("{},{}({:.2f})", COLORS[armor.color], ARMOR_NAMES[armor.name], armor.confidence),
                armor.left.top
            );
        }

        cv::imshow("draw_armor", draw_armor);
        cv::waitKey(1);
    }

    const std::string m_pkgShareDir;
    image_transport::CameraSubscriber m_camSub;
    std::unique_ptr<Detector> m_detector;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}