FROM osrf/ros:noetic-desktop-full

WORKDIR /root/catkin_ws
RUN apt-get update \
    && apt-get install -y python3-pip python3-setuptools python3-yaml ros-noetic-ros-control ros-noetic-ros-controllers python3-catkin-tools \
    && apt-get clean \
    && pip3 install -U pip \
    && update-ca-certificates -f

COPY ./core ./src/core
COPY ./ros/picarx_msgs ./src/picarx_msgs
COPY ./simulation ./src/simulation

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

ENTRYPOINT [ "./src/core/docker/entrypoint.sh" ]