cmake_minimum_required(VERSION 3.8)
project(myrobot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include/)

add_executable(myrobot_driver src/myrobot_driver.cpp)
ament_target_dependencies(myrobot_driver rclcpp geometry_msgs sensor_msgs nav_msgs tf2 tf2_ros std_msgs)

add_executable(myrobot_wheel_fb src/myrobot_wheel_fb.cpp src/oneeurofilter/OneEuroFilter.cpp)
ament_target_dependencies(myrobot_wheel_fb rclcpp std_msgs)

add_executable(myrobot_pwrmon src/myrobot_pwrmon.cpp src/pwrmon/i2c.c src/pwrmon/ina226.c)
ament_target_dependencies(myrobot_pwrmon rclcpp sensor_msgs)

target_include_directories(myrobot_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(myrobot_driver PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS myrobot_driver myrobot_pwrmon myrobot_wheel_fb
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
