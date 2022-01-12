#!/bin/bash
set -e

source /root/catkin_ws/devel/setup.bash
source /root/.bashrc

exec "$@"
