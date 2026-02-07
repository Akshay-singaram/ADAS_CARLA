cmake_minimum_required(VERSION 3.8)
project(space_arm_vision)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python nodes
install(PROGRAMS
  src/visual_servoing_node.py
  src/target_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install config and launch files
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
