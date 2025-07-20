#include <cv_bridge/cv_bridge.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fmt/format.h>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "armor_detector/armordetectornode.hpp"
#include "armor_detector/helpers.hpp"

#define DEBUG 0

ArmorDetectorNode::ArmorDetectorNode()
    : Node("armor_detector"),
      m_pkgShareDir(ament_index_cpp::get_package_share_directory("armor_detector")),
      m_camInfoInitialized(false)
{
    RCLCPP_INFO(get_logger(), "Starting armor_detector node!");

    m_armorsPub = this->create_publisher<armor_interface::msg::Armors>("/detector/armors", rclcpp::SensorDataQoS());

    m_camSub = image_transport::create_camera_subscription(
        this,
        "image_raw",
        std::bind(&ArmorDetectorNode::cameraSubscriptionCallback, 
            this, std::placeholders::_1, std::placeholders::_2),
        "raw"
    );

    // init detector
    DetectorConfig detector_config;
    {
        detector_config.binary_thres = declare_parameter("binary_thres", 120);
        detector_config.lightbar_min_ratio = declare_parameter("lightbar_min_ratio", 2.0);
        detector_config.lightbar_max_ratio = declare_parameter("lightbar_max_ratio", 20.0);
        detector_config.lightbar_angle = declare_parameter("lightbar_angle", 40.0);
    }
    m_detector = std::make_unique<Detector>(m_pkgShareDir, detector_config);

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

void ArmorDetectorNode::cameraSubscriptionCallback(sensor_msgs::msg::Image::ConstSharedPtr image, 
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info)
{
    // 初始化相机参数和PnP
    if (!m_camInfoInitialized)
    {
        m_camInfoInitialized = true;
        m_pnpSolver = std::make_unique<PnPSolver>(info);
        RCLCPP_INFO(get_logger(), "Camera info initialized!");
    }

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
        // 收集装甲板位姿信息并发布
        m_armorsMsg.header = image->header;
        m_armorsMsg.data.clear();
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

                m_armorsMsg.data.emplace_back(armorMsg);
            }
            else
                RCLCPP_WARN(get_logger(), "PnP failed!");
        }
    
        m_armorsPub->publish(m_armorsMsg);
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