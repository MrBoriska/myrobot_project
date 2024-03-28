FROM osrf/ros:humble-desktop-full

RUN apt update
RUN apt install -y libboost-all-dev libi2c-dev \
    ros-humble-foxglove-bridge \
    ros-humble-joint-state-publisher

COPY .  /myrobot_project
WORKDIR "/myrobot_project"

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
RUN echo "source /myrobot_project/install/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=3" >> ~/.bashrc

VOLUME [ "/myrobot_project" ]

CMD ["/bin/bash"]