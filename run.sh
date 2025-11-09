#!/bin/bash

# Run container con:
# - Volume mount per src
# - Privileged mode (per accesso hardware)
# - Host network (stessa rete del PC)
# - ROS_DOMAIN_ID=30
docker run -it --rm \
    -v "$(pwd)/src:/ros2_ws/src" \
    --privileged \
    --network host \
    -e ROS_DOMAIN_ID=30 \
    -e ROS_LOCALHOST_ONLY=0 \
    -e TURTLEBOT3_MODEL=burger \
    camera_rect_img \

