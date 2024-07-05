#!/bin/bash

source /opt/ros/iron/setup.bash
source /app/src/workspace/install/setup.bash

cd /app/src

sudo apt-get update

rosdep install --from-paths /app/src/workspace/src --ignore-src -r -y

ros2 launch ./launch/launch.py
