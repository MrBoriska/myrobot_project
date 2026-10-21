cmake_minimum_required(VERSION 3.8)
project(myrobot_custom_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCL REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(slam_node src/slam_node.cpp src/ieskf_slam.cpp)
target_include_directories(slam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(slam_node PUBLIC c_std_99 cxx_std_17)  # Require C++17

ament_target_dependencies(
  slam_node
  "rclcpp"
  "sensor_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "ament_index_cpp"
)
target_link_libraries(slam_node ${OpenCL_LIBRARIES})

install(TARGETS slam_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  kernels
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
