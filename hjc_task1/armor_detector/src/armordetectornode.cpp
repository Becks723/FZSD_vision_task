#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/format.h>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "armor_detector/armordetectornode.hpp"
#include "armor_detector/helpers.hpp"

ArmorDetectorNode::ArmorDetectorNode()
    : Node("armor_detector"),
      m_pkgShareDir(ament_index_cpp::get_package_share_directory("armor_detector")),
      m_camInfoInitialized(false)
{
    RCLCPP_INFO(get_logger(), "Starting armor_detector node!");

    // 初始化装甲板发布器
    m_armorsPub = this->create_publisher<armor_interface::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

    // 初始化/image_raw话题订阅器
    m_imgSub = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&ArmorDetectorNode::imageSubscriptionCallback, this, std::placeholders::_1)
    );

    // 初始化/camera_info话题订阅器
    m_camSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info",
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr info) {
            if (!m_camInfoInitialized)
            {
                m_camInfoInitialized = true;
                m_pnpSolver = std::make_unique<PnPSolver>(info);
                RCLCPP_INFO(get_logger(), "Camera info initialized!");
            }
        }
    );

    // 初始化Detector
    DetectorConfig detector_config;
    {
        detector_config.binary_thres = declare_parameter("binary_thres", 120);
        detector_config.lightbar_min_ratio = declare_parameter("lightbar_min_ratio", 2.0);
        detector_config.lightbar_max_ratio = declare_parameter("lightbar_max_ratio", 20.0);
        detector_config.lightbar_angle = declare_parameter("lightbar_angle", 40.0);
    }
    m_detector = std::make_unique<Detector>(m_pkgShareDir, detector_config);

    // 初始化装甲板tf播报器
    m_armorBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(this);
}

void ArmorDetectorNode::imageSubscriptionCallback(sensor_msgs::msg::Image::ConstSharedPtr image)
{
    // 通过cv_bridge将 sensor_msgs的image 转成 opencv的Mat
    cv::Mat frame;
    try
    {
        frame = cv_bridge::toCvShare(image, "bgr8")->image;
    }
    catch (const cv_bridge::Exception& ex)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", ex.what());
        return;
    }

    if (frame.empty())
    {
        RCLCPP_INFO(get_logger(), "Null frame!");
        return;
    }

    // 识别装甲板逻辑
    auto armors = m_detector->detect(frame);

    RCLCPP_INFO(get_logger(), "%d armors detected.", armors.size());

    if (m_pnpSolver)
    {
        // 收集 装甲板位姿信息 & tf坐标变换，并发布
        m_armorsMsg.header = image->header;
        m_armorsMsg.data.clear();
        m_armorTransforms.clear();
        int index = 0;
        for (const auto& armor : armors)
        {
            armor_interface::msg::Armor armorMsg;
            armorMsg.color = ARMOR_COLORS[armor.color];
            armorMsg.name = ARMOR_NAMES[armor.name];
            armorMsg.type = ARMOR_TYPES[armor.type];
            
            cv::Mat rvec, tvec;
            bool success = m_pnpSolver->solve(armor, rvec, tvec);
            if (success)
            {
                armorMsg.rvec.x = rvec.at<double>(0);
                armorMsg.rvec.y = rvec.at<double>(1);
                armorMsg.rvec.z = rvec.at<double>(2);
                // tvec 位移
                armorMsg.pose.position.x = tvec.at<double>(0);
                armorMsg.pose.position.y = tvec.at<double>(1);
                armorMsg.pose.position.z = tvec.at<double>(2);

                // rvec 转 旋转矩阵
                cv::Mat rmat;
                cv::Rodrigues(rvec, rmat);

                // 旋转矩阵 转 tf2::Matrix3x3
                tf2::Matrix3x3 tf2rmat(
                    rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
                    rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2),
                    rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2)
                );
                // 提取四元数
                tf2::Quaternion tf2q;
                tf2rmat.getRotation(tf2q);
                armorMsg.pose.orientation.x = tf2q.x();
                armorMsg.pose.orientation.y = tf2q.y();
                armorMsg.pose.orientation.z = tf2q.z();
                armorMsg.pose.orientation.w = tf2q.w();

                // 构建TransformStamped坐标变换
                geometry_msgs::msg::TransformStamped stamped;
                stamped.header = image->header;
                stamped.child_frame_id = fmt::format("armor_{}_frame", index + 1);
                stamped.transform.translation.x = tvec.at<double>(0);
                stamped.transform.translation.y = tvec.at<double>(1);
                stamped.transform.translation.z = tvec.at<double>(2);
                stamped.transform.rotation = armorMsg.pose.orientation;

                m_armorsMsg.data.emplace_back(armorMsg);
                m_armorTransforms.emplace_back(stamped);
            }
            else
                RCLCPP_WARN(get_logger(), "PnP failed!");

            index++;
        }
    
        m_armorsPub->publish(m_armorsMsg);
        m_armorBroadcaster->sendTransform(m_armorTransforms);
    }

    cv::Mat draw_armor = frame.clone();
    for (const Armor& armor : armors)
    {
        helpers::drawPoints(draw_armor, armor.points);
        helpers::drawText(
            draw_armor,
            fmt::format("{},{}({:.2f})", ARMOR_COLORS[armor.color], ARMOR_NAMES[armor.name], armor.confidence),
            armor.left.top
        );
    }

    cv::imshow("draw_armor", draw_armor);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}