#!/bin/bash

xhost local:docker

docker compose -f compose.yml $1 ros