#!/bin/bash

source /opt/ros/iron/setup.bash
source /app/src/workspace/install/setup.bash

cd /app/src

sudo apt-get update

# rosdep update

rosdep install --from-paths /app/src/workspace/src --ignore-src -r -y -q

python3 -m pip install -e ./workspace/src/ProtoRL

ros2 launch ./launch/launch2.py
