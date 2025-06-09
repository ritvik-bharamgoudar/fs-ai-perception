#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "slam_example/image_grabber_stereo_inertial.hpp"
#include "System.h"  // ORB-SLAM3 core system

using std::placeholders::_1;
using std::placeholders::_2;

/// Sync policy: approximate matching of left/right images
using StereoApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

int main(int argc, char **argv)
{
    // Initialise ROS2 node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("orb_slam3_stereo_inertial");

    // --- Parameter handling ---
    std::string vocab_file, settings_file;
    node->declare_parameter("vocab_path", "");
    node->declare_parameter("config_path", "");
    node->get_parameter("vocab_path", vocab_file);
    node->get_parameter("config_path", settings_file);

    if (vocab_file.empty() || settings_file.empty()) {
        RCLCPP_FATAL(node->get_logger(), "vocab_path or config_path is empty! Exiting...");
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Vocab path: %s", vocab_file.c_str());
    RCLCPP_INFO(node->get_logger(), "Config path: %s", settings_file.c_str());

    // --- Create SLAM system ---
    ORB_SLAM3::System SLAM(vocab_file, settings_file, ORB_SLAM3::System::IMU_STEREO, true);

    // --- Create grabber class ---
    auto grabber = std::make_shared<ImageGrabberInertial>(&SLAM);

    // --- Stereo image subscribers ---
    auto qos = rclcpp::SensorDataQoS();  // Matches ZED driver
    node->create_subscription<sensor_msgs::msg::Image>(
        "/zed/left/image_rect_color", rclcpp::SensorDataQoS(),
        std::bind(&ImageGrabberInertial::GrabLeft, grabber.get(), _1));
    
    node->create_subscription<sensor_msgs::msg::Image>(
        "/zed/right/image_rect_color", rclcpp::SensorDataQoS(),
        std::bind(&ImageGrabberInertial::GrabRight, grabber.get(), _1));
    
    // Timer to check for stereo sync ~30Hz
    node->create_wall_timer(
        std::chrono::milliseconds(33),
        std::bind(&ImageGrabberInertial::AttemptManualSync, grabber.get()));
    // Create a message filter for stereo image synchronization    

    // --- IMU subscriber ---
    auto imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
        "/camera/imu/data", rclcpp::SensorDataQoS(),  // High queue depth to match high IMU rate
        std::bind(&ImageGrabberInertial::GrabImu, grabber.get(), _1));

    // --- Start ROS loop ---
    RCLCPP_INFO(node->get_logger(), "ORB-SLAM3 stereo-inertial node started.");
    rclcpp::spin(node);

    // --- Shutdown procedure ---
    RCLCPP_INFO(node->get_logger(), "Shutting down SLAM...");
    SLAM.Shutdown();
    //SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");

    return 0;
}


