################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(myrobot_navigation2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)

################################################################################
# Install
################################################################################
# NOTE: `map/` does not exist in this package yet (it is produced by SLAM or by
# `ros2 run nav2_map_server map_saver_cli`). Listing a missing directory makes
# `colcon build --symlink-install` fail, so only existing ones are installed.
set(NAV2_DIRS launch param rviz)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/map")
  list(APPEND NAV2_DIRS map)
endif()

install(
  DIRECTORY ${NAV2_DIRS}
  DESTINATION share/${PROJECT_NAME}
)

################################################################################
# Macro for ament package
################################################################################
ament_package()
