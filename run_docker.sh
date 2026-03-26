#!/bin/bash

# 获取当前脚本所在目录的绝对路径
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

xhost +local:docker > /dev/null

# 彻底清理旧容器
docker stop tip_obstacle_Dev 2>/dev/null && echo "停止旧容器"
docker rm -f tip_obstacle_Dev 2>/dev/null

docker run -it \
    --name="tip_obstacle_Dev" \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$SCRIPT_DIR/src:/home/ros_ws/src" \
    --volume="$SCRIPT_DIR/workspace:/home/ros_ws/workspace" \
    --net=host \
    --privileged \
    tip_obstacle_img
    