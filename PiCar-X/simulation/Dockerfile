FROM osrf/ros:noetic-desktop-full

WORKDIR /root/catkin_ws

RUN apt-get update \
    && apt-get install -y python3-pip python3-setuptools python3-yaml ros-noetic-ros-control ros-noetic-ros-controllers python3-catkin-tools \
    && apt-get clean \
    && pip3 install -U pip \
    && update-ca-certificates -f

COPY --from=picarx/core:latest /root/catkin_ws/ .
COPY ./simulation ./src/simulation

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin config --isolate-devel --install && catkin build" \
    && touch /root/.bashrc \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

ENTRYPOINT [ "/root/catkin_ws/src/arches/arches_core/ENTRYPOINT.sh" ]