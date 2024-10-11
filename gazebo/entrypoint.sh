#!/bin/bash

source /opt/ros/iron/setup.bash
source /app/src/workspace/install/setup.bash

cd /app/src

sudo apt-get update

rosdep update

# rosdep update

rosdep install --from-paths /app/src/workspace/src --ignore-src -r -y -q

pip3 install billiard

python3 -m pip install -e ./workspace/src/ProtoRL

/app/cmd/build_packages.sh

ros2 launch ./launch/launch.py
