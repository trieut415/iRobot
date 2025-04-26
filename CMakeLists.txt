cmake_minimum_required(VERSION 3.5)
project(p2p_ros_package)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY p2p_ros_package
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()