FROM picarx/core:latest

COPY . ./src/skills/ackermann_drive

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

ENTRYPOINT [ "./src/core/docker/entrypoint.sh" ]