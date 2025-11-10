FROM ros:humble-ros-base

# Installa dipendenze essenziali
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Installa dipendenze Python
#RUN pip3 install --no-cache-dir torch torchvision opencv-python numpy pyyaml

# Crea workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Copia SOLO package.xml per rosdep (layer caching)
COPY src/camera_rectifier/package.xml ./src/camera_rectifier/

# Installa dipendenze ROS2 con rosdep
RUN apt-get update && \
    rosdep update && \
    rosdep install -r --from-paths src -i -y --rosdistro humble && \
    rm -rf /var/lib/apt/lists/*

# Copia tutto il sorgente
COPY src ./src

# Build del workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Setup automatico
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Entrypoint per auto-source
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/humble/setup.bash\n\
source /ros2_ws/install/setup.bash\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
