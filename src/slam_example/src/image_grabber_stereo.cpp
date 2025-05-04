#include "slam_example/image_grabber_stereo.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

ImageGrabberStereo::ImageGrabberStereo(std::shared_ptr<ORB_SLAM3::System> pSLAM,
                                       rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr rospub,
                                       std::shared_ptr<rclcpp::Node> ros_node,
                                       const std::string& camera_frame)
    : mpSLAM(pSLAM), odom_pub_(rospub), rosNode_(ros_node), tf_frame_(camera_frame)
{}

cv::Mat ImageGrabberStereo::ConvertToGray(const sensor_msgs::msg::Image::SharedPtr &img_msg)
{
    try {
        auto cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
        return cv_ptr->image.clone();
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rosNode_->get_logger(), "cv_bridge exception: %s", e.what());
        return cv::Mat();
    }
}

void ImageGrabberStereo::GrabStereo(const sensor_msgs::msg::Image::SharedPtr left_msg,
                                     const sensor_msgs::msg::Image::SharedPtr right_msg)
{
    cv::Mat imLeft = ConvertToGray(left_msg);
    cv::Mat imRight = ConvertToGray(right_msg);

    if (imLeft.empty() || imRight.empty()) {
        RCLCPP_WARN(rosNode_->get_logger(), "Empty stereo image received.");
        return;
    }

    // Use the timestamp from the left image
    double timestamp = left_msg->header.stamp.sec + left_msg->header.stamp.nanosec * 1e-9;

    Sophus::SE3f pose;
    try {
        pose = mpSLAM->TrackStereo(imLeft, imRight, timestamp);
        PublishPose(pose, left_msg->header.stamp);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rosNode_->get_logger(), "ORB-SLAM3 exception: %s", e.what());
    }
}

void ImageGrabberStereo::PublishPose(const Sophus::SE3f& se3, rclcpp::Time stamp)
{
    Eigen::Vector3f t = se3.translation();
    Eigen::Quaternionf q(se3.rotationMatrix());

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = tf_frame_;

    odom_msg.pose.pose.position.x = t.x();
    odom_msg.pose.pose.position.y = t.y();
    odom_msg.pose.pose.position.z = t.z();

    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_pub_->publish(odom_msg);
}
