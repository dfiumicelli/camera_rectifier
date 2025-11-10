#!/bin/bash

# Run container con:
# - Volume mount per src
# - Privileged mode (per accesso hardware)
# - Host network (stessa rete del PC)
# - ROS_DOMAIN_ID=30
docker run -d --rm \
    --privileged \
    --network host \
    -e ROS_DOMAIN_ID=30 \
    -e ROS_LOCALHOST_ONLY=0 \
    --name camera_rect_container \
    camera_rect_img \
    bash -c "ros2 run camera_rectifier camera_rect_node"

echo "✓ Container avviato in background"
echo "Logs: docker logs -f camera_rect_container"
echo "Stop: docker stop camera_rect_container"
