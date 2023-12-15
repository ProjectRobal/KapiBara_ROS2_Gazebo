#!/bin/bash

xhost local:docker

arg=$1


if [ "$arg" = "start" ]; then

docker compose -f compose.yml up

elif [ "$arg" = "build" ]; then

docker compose -f compose.yml build

elif [ "$arg" = "run" ]; then

docker compose -f compose.yml exec ros run_cmd.sh "${@:2}"

elif [ "$arg" = "exec" ]; then

docker compose -f compose.yml exec ros "${@:2}"

elif [ "$arg" = "debug" ]; then

docker compose -f compose.yml exec ros bash

elif [ "$arg" = "gazebo" ]; then

docker compose -f compose.yml exec ros run_cmd.sh ros2 launch gazebo_ros gazebo.launch.py

elif [ "$arg" = "topics" ]; then

docker compose -f compose.yml exec ros run_cmd.sh ros2 topic list -t

elif [ "$arg" = "echo" ]; then

docker compose -f compose.yml exec ros run_cmd.sh ros2 topic echo "${@:2}"

elif [ "$arg" = "info" ]; then

docker compose -f compose.yml exec ros run_cmd.sh ros2 topic info "${@:2}"

elif [ "$arg" = "publish" ]; then

docker compose -f compose.yml exec ros run_cmd.sh ros2 topic pub --once "${@:2}"

fi