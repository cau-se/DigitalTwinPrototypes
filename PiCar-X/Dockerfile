FROM ros:noetic

WORKDIR /root/catkin_ws/
RUN apt update -y \
    && apt-get install -y python3-pip python3-setuptools python3-catkin-tools \
    && apt-get clean \
    && update-ca-certificates -f \
    && pip3 install --upgrade pip

COPY ./core ./src/core
COPY ./ros/picarx_msgs ./src/picarx_msgs

RUN pip3 install -e ./src/core/picarx \
    && /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build" \
    && mv ./src/core/docker/.bashrc /root/.bashrc && chmod +x /root/.bashrc

ENTRYPOINT [ "./src/core/docker/entrypoint.sh" ]

