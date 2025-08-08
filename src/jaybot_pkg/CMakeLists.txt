cmake_minimum_required(VERSION 3.8)
project(jaybot_pkg)

# Enable Python
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SonarStatus.msg"
  DEPENDENCIES std_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install scripts
install(PROGRAMS
  sonars/sonar_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export
ament_export_dependencies(rosidl_default_runtime)
ament_package()
