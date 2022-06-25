FROM git.geomar.de:11411/open-source/arches/arches_core:1-0-0

COPY ./core ./src/core
COPY ./ros/picarx_msgs ./src/picarx_msgs

RUN pip3 install -e ./src/core/picarx \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

ENTRYPOINT [ "/root/catkin_ws/src/arches/arches_core/ENTRYPOINT.sh" ]

