FROM osrf/ros:noetic-desktop-full as gazebo-full
ARG TAG
WORKDIR /root/catkin_ws
RUN apt-get update \
    && apt-get install -y python3-pip python3-setuptools python3-yaml ros-noetic-ros-control ros-noetic-ros-controllers python3-catkin-tools \
    && apt-get clean \
    && update-ca-certificates -f

FROM gazebo:libgazebo11 as gazebo-only
ARG TAG   
WORKDIR /root/catkin_ws
RUN apt-get update \
    && apt-get install -y python3-pip python3-setuptools python3-yaml ros-noetic-ros-control ros-noetic-ros-controllers python3-catkin-tools \
    && apt-get clean \
    && update-ca-certificates -f

ARG TAG=latest

FROM gazebo-full as desktop
ARG TAG
COPY --from=abarbie/picarx:${TAG} /root/catkin_ws/ .
COPY ./simulation ./src/simulation

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin config --isolate-devel --install && catkin build" \
    && touch /root/.bashrc \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

ENTRYPOINT [ "/root/catkin_ws/src/arches/arches_core/ENTRYPOINT.sh" ]


ARG TAG=latest
FROM gazebo-only as tests
COPY --from=abarbie/picarx:${TAG} /root/catkin_ws/ .
COPY ./simulation ./src/simulation

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin config --isolate-devel --install && catkin build" \
    && touch /root/.bashrc \
    && echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

ENTRYPOINT [ "/root/catkin_ws/src/arches/arches_core/ENTRYPOINT.sh" ]