#!/bin/bash

source /opt/ros/iron/setup.bash
source /app/src/workspace/install/setup.bash

cd /app/src

ros2 launch ./launch/launch.py
