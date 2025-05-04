cmake_minimum_required(VERSION 3.5)
project(cone_detector)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedCone3D.msg"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()

