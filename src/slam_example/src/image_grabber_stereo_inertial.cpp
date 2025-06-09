#include "slam_example/image_grabber_stereo_inertial.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <rclcpp/rclcpp.hpp>

/// Max size of IMU queue to prevent unbounded growth
constexpr size_t MAX_IMU_QUEUE_SIZE = 200;

/// Stereo image callback, called when left and right images are approximately synchronized
void ImageGrabberInertial::GrabStereo(
    const sensor_msgs::msg::Image::ConstSharedPtr &left,
    const sensor_msgs::msg::Image::ConstSharedPtr &right)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    // Log encodings for debug
    RCLCPP_INFO(rclcpp::get_logger("GrabStereo"), "Left encoding: %s, Right encoding: %s",
                left->encoding.c_str(), right->encoding.c_str());

    // Validate encodings bgr8
    if (left->encoding != "bgr8" || right->encoding != "bgr8") {
        RCLCPP_WARN(rclcpp::get_logger("GrabStereo"), "Unexpected encodings: left=%s, right=%s",
                    left->encoding.c_str(), right->encoding.c_str());
        return;
    }

    // Check for empty image data
    if (left->data.empty() || right->data.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("GrabStereo"), "Received empty image data");
        return;
    }

    // Store latest image pair
    m_leftImage = left;
    m_rightImage = right;

    // Try tracking if IMU data is available
    TryTrackStereoIfReady();
}

void ImageGrabberInertial::GrabLeft(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    RCLCPP_INFO(rclcpp::get_logger("GrabLeft"), "Got LEFT image at t=%.6f", 
    rclcpp::Time(msg->header.stamp).seconds());
    std::lock_guard<std::mutex> lock(m_mutex);
    m_leftBuffer.push_back(msg);
    if (m_leftBuffer.size() > 30) m_leftBuffer.pop_front();
}

void ImageGrabberInertial::GrabRight(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    RCLCPP_INFO(rclcpp::get_logger("GrabRight"), "Got RIGHT image at t=%.6f", 
    rclcpp::Time(msg->header.stamp).seconds());
    std::lock_guard<std::mutex> lock(m_mutex);
    m_rightBuffer.push_back(msg);
    if (m_rightBuffer.size() > 30) m_rightBuffer.pop_front();
}

void ImageGrabberInertial::AttemptManualSync() {
    RCLCPP_INFO(rclcpp::get_logger("ManualSync"), 
    "Attempting sync: left=%lu, right=%lu", m_leftBuffer.size(), m_rightBuffer.size());

    std::lock_guard<std::mutex> lock(m_mutex);
    const double max_time_diff = 0.05;  // 50 ms

    while (!m_leftBuffer.empty() && !m_rightBuffer.empty()) {
        double t_left = rclcpp::Time(m_leftBuffer.front()->header.stamp).seconds();
        double t_right = rclcpp::Time(m_rightBuffer.front()->header.stamp).seconds();

        double dt = t_left - t_right;
        RCLCPP_INFO(rclcpp::get_logger("ManualSync"),
            "Matching left t=%.6f and right t=%.6f (dt=%.3f)", t_left, t_right, dt);


        if (std::abs(dt) < max_time_diff) {
            m_leftImage = m_leftBuffer.front();
            m_rightImage = m_rightBuffer.front();
            m_leftBuffer.pop_front();
            m_rightBuffer.pop_front();
            TryTrackStereoIfReady();
            return;
        }

        // Drop older frame
        if (dt < 0) m_leftBuffer.pop_front();  // left older
        else        m_rightBuffer.pop_front(); // right older
    }
}


/// IMU callback: appends the new IMU message to the queue
void ImageGrabberInertial::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu)
{
    
    double timu = rclcpp::Time(imu->header.stamp).seconds();

    RCLCPP_INFO(rclcpp::get_logger("GrabImu"), "Received IMU t = %.6f", timu);


    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_imuQueue.size() > MAX_IMU_QUEUE_SIZE) {
        RCLCPP_WARN(rclcpp::get_logger("GrabImu"), "IMU queue too large (%lu), discarding oldest", m_imuQueue.size());
        m_imuQueue.pop_front();
    }

    m_imuQueue.push_back(imu);
}


/// Attempts to track a stereo-inertial frame if stereo images and IMU data are ready
void ImageGrabberInertial::TryTrackStereoIfReady()
{
    RCLCPP_INFO(rclcpp::get_logger("TryTrackStereo"), "Entered TryTrackStereoIfReady()");
    std::lock_guard<std::mutex> lock(m_mutex);

    if (!m_leftImage || !m_rightImage) {
        RCLCPP_WARN(rclcpp::get_logger("TryTrackStereo"), "Missing stereo image(s)");
        return;
    }

    // Convert ROS images to OpenCV
    cv::Mat imLeft, imRight;
    try {
        imLeft = cv_bridge::toCvShare(m_leftImage, "bgr8")->image;
        imRight = cv_bridge::toCvShare(m_rightImage, "bgr8")->image;

        if (imLeft.empty() || imRight.empty()) {
            RCLCPP_WARN(rclcpp::get_logger("TryTrackStereo"), "OpenCV image(s) empty after conversion");
            return;
        }
    } catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("TryTrackStereo"), "cv_bridge conversion error: %s", e.what());
        return;
    }

    // Timestamp of this stereo frame
    double tframe = rclcpp::Time(m_leftImage->header.stamp).seconds();


    RCLCPP_INFO(rclcpp::get_logger("TryTrackStereo"), "Tracking stereo frame at t = %.6f", tframe);
    RCLCPP_INFO(rclcpp::get_logger("TryTrackStereo"), "IMU queue size before filtering: %lu", m_imuQueue.size());

    RCLCPP_INFO(rclcpp::get_logger("TryTrackStereo"), "Stereo frame t = %.6f", tframe);

    // Get all IMU messages before this timestamp
    std::vector<ORB_SLAM3::IMU::Point> vImuMeasurements;

    while (!m_imuQueue.empty()) {
        auto imu = m_imuQueue.front();

        double timu = rclcpp::Time(imu->header.stamp).seconds();

        RCLCPP_INFO(rclcpp::get_logger("TryTrackStereo"), "  IMU t = %.6f", timu);
        RCLCPP_INFO(rclcpp::get_logger("TryTrackStereo"), "Stereo frame t = %.6f", tframe);


        const double max_imu_delay = 0.5;  // seconds
        if (timu > tframe + max_imu_delay) {
            RCLCPP_WARN(rclcpp::get_logger("TryTrackStereo"), 
                "Skipping IMU t=%.6f as it's beyond frame time %.6f + %.2fs buffer",
                timu, tframe, max_imu_delay);
            break;
        }


        cv::Point3f acc(imu->linear_acceleration.x,
                        imu->linear_acceleration.y,
                        imu->linear_acceleration.z);

        cv::Point3f gyro(imu->angular_velocity.x,
                        imu->angular_velocity.y,
                        imu->angular_velocity.z);

        vImuMeasurements.emplace_back(acc, gyro, timu);

        m_imuQueue.pop_front();  // 
    }


    if (vImuMeasurements.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("TryTrackStereo"), "No usable IMU measurements for frame at t = %.6f", tframe);
        return;
    }

    RCLCPP_INFO(rclcpp::get_logger("TryTrackStereo"), "Calling SLAM->TrackStereo...");

    try {
        m_SLAM->TrackStereo(imLeft, imRight, tframe, vImuMeasurements);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("TryTrackStereo"), "Exception during SLAM tracking: %s", e.what());
    }

    m_leftImage.reset();
    m_rightImage.reset();
}




