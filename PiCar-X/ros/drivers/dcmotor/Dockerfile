FROM picarx/core:latest

COPY . ./src/drivers/dcmotor

RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin build"

ENTRYPOINT [ "./src/core/docker/entrypoint.sh" ]