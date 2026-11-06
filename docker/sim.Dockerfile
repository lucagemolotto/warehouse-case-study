# Simulation side: Gazebo Harmonic + Nav2 + TurtleBot3, ROS 2 Jazzy.
FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2 \
      ros-jazzy-nav2-bringup \
      ros-jazzy-nav2-minimal-tb3-sim \
      python3-colcon-common-extensions \
      mesa-utils \
    && rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=waffle
# Keep gz-transport discovery on loopback and away from other gz sims on the host.
ENV GZ_IP=127.0.0.1
ENV GZ_PARTITION=warehouse
WORKDIR /ws
COPY docker/sim_entrypoint.sh /sim_entrypoint.sh
ENTRYPOINT ["/sim_entrypoint.sh"]
CMD ["bash"]
