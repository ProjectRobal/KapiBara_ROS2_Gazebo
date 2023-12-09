#!/bin/bash

xhost local:docker

arg=$1


if [ "$arg" = "start" ]; then

docker compose -f compose.yml up

elif [ "$arg" = "build" ]; then

docker compose -f compose.yml build

elif [ "$arg" = "run" ]; then

docker compose -f compose.yml run ros "${@:2}"

fi