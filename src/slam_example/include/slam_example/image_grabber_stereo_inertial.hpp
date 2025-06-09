#ifndef IMAGE_GRABBER_STEREO_INERTIAL_H
#define IMAGE_GRABBER_STEREO_INERTIAL_H

#include <deque>
#include <mutex>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "System.h"
#include <rclcpp/time.hpp>

/// \brief Handles stereo image and IMU synchronization and feeds them to ORB-SLAM3
class ImageGrabberInertial {
public:
    /// Constructor: stores pointer to the SLAM system
    ImageGrabberInertial(ORB_SLAM3::System* pSLAM)
        : m_SLAM(pSLAM), m_leftImage(nullptr), m_rightImage(nullptr) {}

    /// \brief Callback for synchronized stereo images
    void GrabStereo(const sensor_msgs::msg::Image::ConstSharedPtr& left,
                    const sensor_msgs::msg::Image::ConstSharedPtr& right);

    /// \brief Callback for IMU messages
    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void GrabLeft(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void GrabRight(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void AttemptManualSync();  // called from a timer

private:
    /// \brief Triggers ORB-SLAM3 tracking if stereo and IMU data are ready
    void TryTrackStereoIfReady();

    ORB_SLAM3::System* m_SLAM;  ///< Pointer to SLAM system
    sensor_msgs::msg::Image::ConstSharedPtr m_leftImage;  ///< Latest left image
    sensor_msgs::msg::Image::ConstSharedPtr m_rightImage; ///< Latest right image
    std::deque<sensor_msgs::msg::Imu::SharedPtr> m_imuQueue; ///< Buffered IMU messages
    std::mutex m_mutex; ///< Mutex for thread-safe access to shared data
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> m_leftBuffer;
    std::deque<sensor_msgs::msg::Image::ConstSharedPtr> m_rightBuffer;
};

#endif // IMAGE_GRABBER_STEREO_INERTIAL_H

