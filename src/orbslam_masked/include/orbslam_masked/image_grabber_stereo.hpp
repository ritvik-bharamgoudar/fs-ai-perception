#ifndef IMAGE_GRABBER_STEREO_HPP
#define IMAGE_GRABBER_STEREO_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "System.h"  // ORB-SLAM3 core

#include <queue>
#include <mutex>
#include <memory>

class ImageGrabberStereo : public std::enable_shared_from_this<ImageGrabberStereo>
{
public:
    ImageGrabberStereo(std::shared_ptr<ORB_SLAM3::System> pSLAM,
                       rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
                       std::shared_ptr<rclcpp::Node> ros_node,
                       const std::string& camera_frame);

    void GrabStereo(const sensor_msgs::msg::Image::SharedPtr left_msg,
                    const sensor_msgs::msg::Image::SharedPtr right_msg);

    cv::Mat ConvertToGray(const sensor_msgs::msg::Image::SharedPtr &img_msg);
    void PublishPose(const Sophus::SE3f& se3, rclcpp::Time stamp);

private:
    std::shared_ptr<ORB_SLAM3::System> mpSLAM;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<rclcpp::Node> rosNode_;
    std::string tf_frame_;
    cv_bridge::CvImagePtr cv_ptr_left_, cv_ptr_right_;
};

#endif // IMAGE_GRABBER_STEREO_HPP
