# Development / simulation image for myrobot_project.
#
# The workspace is *mounted*, not baked in, so the image stays small and the
# same container serves the real robot, the Gazebo simulation and the offline
# synthetic-sensor debug loop. The entry point builds the workspace on demand.
#
#   ./create_docker.sh build
#   ./create_docker.sh shell
#   ./create_docker.sh sim        # synthetic sensors + SLAM, headless
#   ./create_docker.sh check      # synthetic sensors + SLAM + accuracy report
#   ./create_docker.sh gazebo     # Gazebo (server only) + SLAM

FROM osrf/ros:humble-desktop-full

ARG DEBIAN_FRONTEND=noninteractive

# ros-humble-desktop-full already brings rviz2, rosbag2, tf2, rclcpp, gazebo_ros
# and the common message packages. What is missing for this project:
#   libi2c-dev            ros2_mpu9250_driver (i2c/smbus.h)
#   libboost-all-dev      xv_11_driver (boost::asio serial)
#   libeigen3-dev         myrobot_custom_slam
#   xacro                 urdf/myrobot.xacro is generated at launch time
#   gazebo-ros-pkgs       gzserver/gzclient/spawn_entity launch files
#   gazebo-plugins        libgazebo_ros_diff_drive / ray_sensor / imu_sensor
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        curl \
        less \
        nano \
        tmux \
        libboost-all-dev \
        libi2c-dev \
        libeigen3-dev \
        python3-numpy \
        python3-pip \
        python3-colcon-common-extensions \
        ros-humble-xacro \
        ros-humble-joint-state-publisher \
        ros-humble-robot-state-publisher \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-plugins \
        ros-humble-foxglove-bridge \
        ros-humble-teleop-twist-keyboard \
        ros-humble-tf2-tools \
        ros-humble-topic-tools \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=3
ENV WORKSPACE=/myrobot_project
# Build artefacts for the mounted workspace live outside of it (see the
# entrypoint): the host must never share build/install with the container.
ENV WS_BUILD=/opt/ws/build
ENV WS_INSTALL=/opt/ws/install
VOLUME [ "/opt/ws" ]

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR ${WORKSPACE}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
