cmake_minimum_required(VERSION 3.8)
project(myrobot_custom_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------- core library
# ROS-free so that it can be unit tested, benchmarked and reused without rclcpp.
add_library(myrobot_slam_core
  src/voxel_map.cpp
  src/eskf.cpp
)
target_include_directories(myrobot_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(myrobot_slam_core PUBLIC Eigen3::Eigen)
target_compile_features(myrobot_slam_core PUBLIC cxx_std_17)

# ----------------------------------------------------------------- ROS node
add_executable(slam_node src/slam_node.cpp)
target_link_libraries(slam_node myrobot_slam_core)
ament_target_dependencies(slam_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)

# ------------------------------------------------------------ benchmark tool
add_executable(bench_slam bench/bench_slam.cpp)
target_link_libraries(bench_slam myrobot_slam_core)

install(TARGETS slam_node bench_slam
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY launch config rviz worlds
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/fake_sensors.py
  scripts/check_slam_accuracy.py
  scripts/circle_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_voxel_map test/test_voxel_map.cpp)
  target_link_libraries(test_voxel_map myrobot_slam_core)

  ament_add_gtest(test_eskf test/test_eskf.cpp)
  target_link_libraries(test_eskf myrobot_slam_core)

  ament_add_gtest(test_registration test/test_registration.cpp)
  target_link_libraries(test_registration myrobot_slam_core)
endif()

ament_package()
