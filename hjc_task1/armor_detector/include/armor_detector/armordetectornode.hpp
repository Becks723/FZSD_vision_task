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

/**
 * @brief 装甲板识别节点，负责 1.订阅图像、相机数据 2. 识别装甲板 3. 将识别好的数据发布（装甲板位姿+tf2相机到装甲板坐标系变换）
 */
class ArmorDetectorNode : public rclcpp::Node
{
public:
    ArmorDetectorNode();

private:
    void imageSubscriptionCallback(sensor_msgs::msg::Image::ConstSharedPtr image);
    
    const std::string m_pkgShareDir;
    /* 图像订阅器 */
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_imgSub;
    /* 相机参数订阅器 */
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr m_camSub;
    /* 装甲板消息发布器 */
    rclcpp::Publisher<armor_interface::msg::Armors>::SharedPtr m_armorsPub;
    /* 识别逻辑 */
    std::unique_ptr<Detector> m_detector;
    /* 解算PnP逻辑 */
    std::unique_ptr<PnPSolver> m_pnpSolver;
    bool m_camInfoInitialized;

    /* 自定义的装甲板消息 */
    armor_interface::msg::Armors m_armorsMsg;

    /* tf2坐标变换广播器 */
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_armorBroadcaster;
    /* tf2坐标变换 */
    std::vector<geometry_msgs::msg::TransformStamped> m_armorTransforms;
};

#endif