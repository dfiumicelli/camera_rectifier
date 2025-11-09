FROM ros:humble-ros-base

# Installa SOLO essenziale per lo sviluppo
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    ros-humble-cv-bridge \
    ros-humble-sensor-msgs \
    ros-humble-image-transport \
    && rm -rf /var/lib/apt/lists/*

# Minimal Python deps
#RUN pip3 install --no-cache-dir opencv-python numpy

# Setup workspace
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Auto-source su bash
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
