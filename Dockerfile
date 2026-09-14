FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 1. Install everything the project needs
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-humble-turtlebot3 \
      ros-humble-turtlebot3-msgs \
      ros-humble-turtlebot3-simulations \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-navigation2 \
      ros-humble-nav2-bringup \
      ros-humble-nav2-simple-commander \
      ros-humble-slam-toolbox \
      ros-humble-cv-bridge \
      ros-humble-tf2-geometry-msgs \
      python3-opencv \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 2. Stop the build if the old ArUco API (OpenCV 4.5.4) is missing
RUN python3 -c "import cv2; cv2.aruco.Dictionary_get; cv2.aruco.estimatePoseSingleMarkers; print('cv2', cv2.__version__)"

# 3. Copy your packages in and build them
WORKDIR /sar_ws
COPY src ./src
RUN source /opt/ros/humble/setup.bash && colcon build --symlink-install

# 4. Entrypoint: runs every time the container starts
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && \
    echo 'source /entrypoint.sh' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
