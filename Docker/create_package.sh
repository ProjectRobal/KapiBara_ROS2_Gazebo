#!/bin/bash

source /opt/ros/foxy/setup.bash

cd /app/src/workspace

mkdir src

cd src

ros2 pkg create --build-type ament_cmake $1