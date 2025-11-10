#!/bin/bash


xhost +local:docker

docker run -d --rm \
    --name camera_rectifier \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v ~/.Xauthority:/root/.Xauthority:rw \
    camera_rect_img \
    ros2 run camera_rectifier camera_rect_node

echo "✓ Container camera_rectifier avviato in background"
echo "Logs: docker logs -f camera_rectifier"
