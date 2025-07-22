#ifndef _ARMOR_DETECTOR_NODE_
#define _ARMOR_DETECTOR_NODE_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
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
    void imageSubscriptionCallback(sensor_msgs::msg::Image::ConstSharedPtr image);
    
    const std::string m_pkgShareDir;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_imgSub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_camSub;
    rclcpp::Publisher<armor_interface::msg::Armors>::SharedPtr m_armorsPub;
    std::unique_ptr<Detector> m_detector;
    std::unique_ptr<PnPSolver> m_pnpSolver;
    bool m_camInfoInitialized;

    armor_interface::msg::Armors m_armorsMsg;

    std::unique_ptr<tf2_ros::TransformBroadcaster> m_armorBroadcaster;
    std::vector<geometry_msgs::msg::TransformStamped> m_armorTransforms;
};

#endif