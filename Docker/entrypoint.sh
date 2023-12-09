#!/bin/bash
#Xvfb :0 -screen 0 1366x768x16 &

#x11vnc -passwd TestVNC -forever -display :0 &

/usr/sbin/sshd -p 3000

source /opt/ros/foxy/setup.bash
source /app/src/workspace/install/setup.bash

apt update -y

xargs apt -y install < /app/dep/packages.txt

python3 -m pip install -r /app/dep/python.txt

ros2 launch /app/src/launch/launch.py
