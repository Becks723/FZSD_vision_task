#ifndef _ARMOR_DETECTOR_NODE_
#define _ARMOR_DETECTOR_NODE_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <opencv2/opencv.hpp>

#include "detector.hpp"
#include "helpers.hpp"
#include "pnpsolver.hpp"
#include "armor_interface/msg/armor.hpp"
#include "armor_interface/msg/armors.hpp"

class ArmorDetectorNode : public rclcpp::Node
{
public:
    ArmorDetectorNode();

private:
    void cameraSubscriptionCallback(sensor_msgs::msg::Image::ConstSharedPtr image,
        sensor_msgs::msg::CameraInfo::ConstSharedPtr info);

    const std::string m_pkgShareDir;
    image_transport::CameraSubscriber m_camSub;
    rclcpp::Publisher<armor_interface::msg::Armors>::SharedPtr m_armorsPub;
    std::unique_ptr<Detector> m_detector;
    std::unique_ptr<PnPSolver> m_pnpSolver;
    bool m_camInfoInitialized;

    armor_interface::msg::Armors m_armorsMsg;
};

#endif