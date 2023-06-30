#!/bin/bash
 
set -e

# Ros build
source "/opt/ros/foxy/setup.bash"

echo "==============HDL localization ROS2 Docker Env Ready================"

cd /root/catkin_ws

exec "$@"
