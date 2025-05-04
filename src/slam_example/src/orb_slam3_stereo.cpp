#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include "slam_example/image_grabber_stereo.hpp"
#include "System.h"

using namespace message_filters;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("orb_slam3_stereo_node");

    // Load parameters
    std::string vocab_path = node->declare_parameter<std::string>("vocab_path", "");
    std::string config_path = node->declare_parameter<std::string>("config_path", "");
    std::string camera_frame = node->declare_parameter<std::string>("camera_frame", "camera_link");
    bool show_viewer = node->declare_parameter<bool>("viewer", false);

    // Initialise ORB-SLAM3 system
    auto slam_system = std::make_shared<ORB_SLAM3::System>(
        vocab_path, config_path, ORB_SLAM3::System::STEREO, show_viewer);

    // Publisher for pose
    auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/odometry/slam", 10);

    // Create ImageGrabber
    auto grabber = std::make_shared<ImageGrabberStereo>(slam_system, odom_pub, node, camera_frame);

    // Stereo subscriptions
    auto left_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, "/zed/left/image_rect_color");
    auto right_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(node, "/zed/right/image_rect_color");

    typedef sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> StereoSyncPolicy;
    auto sync = std::make_shared<Synchronizer<StereoSyncPolicy>>(StereoSyncPolicy(10), *left_sub, *right_sub);
    sync->registerCallback(&ImageGrabberStereo::GrabStereo, grabber.get());


    RCLCPP_INFO(node->get_logger(), "ORB-SLAM3 Stereo node started.");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
