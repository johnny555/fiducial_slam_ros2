cmake_minimum_required(VERSION 3.8)
project(fiducial_slam_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fiducial_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)

##############
## Services ##
##############

# Generate services in the 'srv' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddFiducial.srv"
  DEPENDENCIES std_msgs
)

###########
## Build ##
###########

# Build the fiducial_slam_node executable
add_executable(fiducial_slam_node 
  src/fiducial_slam_node.cpp
  src/transform_with_variance.cpp
  src/map.cpp
)

# Include directories
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link against dependencies
ament_target_dependencies(fiducial_slam_node
  rclcpp
  fiducial_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  OpenCV
)

# Link against the generated interfaces (updated to use recommended approach)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fiducial_slam_node "${cpp_typesupport_target}")

# Install targets
install(TARGETS
  fiducial_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_package()
