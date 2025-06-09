cmake_minimum_required(VERSION 3.8)
project(slam_example)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(message_filters REQUIRED) 

# Add include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  /usr/local/include/eigen3
  /usr/local/include
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_stereo_inertial/include
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_stereo_inertial/include/CameraModels
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_stereo_inertial/Thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_stereo_inertial/Thirdparty/DBoW2/DBoW2
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_stereo_inertial/Thirdparty/g2o
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_stereo_inertial
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_backup/include
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_backup/include/CameraModels
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_backup/Thirdparty/Sophus
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_backup/Thirdparty/DBoW2/DBoW2
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_backup/Thirdparty/g2o
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_backup
)

# Link libraries
link_directories(${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_backup/lib)

add_executable(orb_slam3_stereo
  src/orb_slam3_stereo.cpp
  src/image_grabber_stereo.cpp
)

target_link_libraries(orb_slam3_stereo
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  /usr/local/lib/libpangolin.so
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_backup/lib/libORB_SLAM3.so
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_backup/Thirdparty/DBoW2/lib/libDBoW2.so
  -lGL
  -lGLU
  -lGLEW
)

ament_target_dependencies(orb_slam3_stereo
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  message_filters
)

#Install the node
install(TARGETS orb_slam3_stereo
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(orb_slam3_stereo_inertial
  src/orb_slam3_stereo_inertial.cpp
  src/image_grabber_stereo_inertial.cpp
)

target_link_libraries(orb_slam3_stereo_inertial
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
  ${message_filters_LIBRARIES}
  /usr/local/lib/libpangolin.so
  ${PROJECT_SOURCE_DIR}/../../ORB_SLAM3_stereo_inertial/lib/libORB_SLAM3.so
  -lGL
  -lGLU
  -lGLEW
)

ament_target_dependencies(orb_slam3_stereo_inertial
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  message_filters
)

install(TARGETS orb_slam3_stereo_inertial
  DESTINATION lib/${PROJECT_NAME}
)


# Install config and launch files
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

