#!/bin/bash

source /opt/ros/foxy/setup.bash

ros2 launch /app/src/launch/launch.py
ros2 topic list