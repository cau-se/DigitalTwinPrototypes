FROM osrf/ros:noetic-desktop-full

RUN apt-get update \
    && apt-get install -y python3-yaml ros-noetic-ros-control ros-noetic-ros-controllers python3-catkin-tools \
    && apt-get clean \
    && update-ca-certificates -f

WORKDIR /root/catkin_ws
COPY ./picarx_control ./src/picarx_control
COPY ./picarx_description ./src/picarx_description
COPY ./picarx_gazebo ./src/picarx_gazebo

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

CMD ["/bin/bash"]